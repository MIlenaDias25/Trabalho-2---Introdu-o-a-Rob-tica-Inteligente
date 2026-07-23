#! /usr/bin/env python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import random

TARGETS = [
    ("verde",    2.20,  2.20),
    ("vermelho", 2.15, -2.15),
    ("azul",    -2.16, -2.16),
    ("laranja", -2.00,  1.20),
]
HOME = (-2.0, 2.0)

# --- Estados ---
PLANNING = 'PLANNING'
FOLLOWING_PATH = 'FOLLOWING_PATH'
RECOVERING = 'RECOVERING'
DONE = 'DONE'


class RRTNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class NavigatorNode(Node):
    def __init__(self):
        super().__init__('navigator_node')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)

        # Pose Atual
        self.x = self.y = self.yaw = 0.0

        # Leitura do LIDAR
        self.scan_ranges = None
        self.scan_n = 0
        self.scan_angle_min = 0.0
        self.scan_angle_increment = 0.0
        self.front_min = float('inf')

        # Máquina de Estados
        self.state = PLANNING
        self.target_idx = 0
        self.going_home = False

        # Caminho do RRT
        self.path = []
        self.current_wp_idx = 0

        # Parâmetros de Controle Cinemático 
        self.ARRIVAL_THRESHOLD = 0.35      
        self.YAW_ALIGN_THRESHOLD = 0.25 
        self.MAX_LINEAR_SPEED = 0.16
        self.MAX_ANGULAR_SPEED = 1.0
        self.kp_linear = 0.5
        self.kp_angular = 1.2

        # Parâmetros do RRT 
        self.ROBOT_RADIUS = 0.12          # checagem de colisão
        self.RRT_STEP_SIZE = 0.25         # Tamanho do passo de expansão da árvore
        self.RRT_MAX_ITER = 3000          # Limite de iterações para evitar loops infinitos
        self.MAP_BOUNDS = (-2.5, 2.5)     # Limites aproximados do cenário (X e Y)

        # Rede de Segurança Traseira 
        self.EMERGENCY_DISTANCE = 0.22
        self.BACKUP_SPEED = 0.08
        self.MAX_BACKUP_DURATION = 2.0
        self.recover_start_time = None

        self.timer = self.create_timer(0.1, self.control_loop)

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def scan_cb(self, msg):
        self.scan_ranges = msg.ranges
        self.scan_n = len(msg.ranges)
        self.scan_angle_min = msg.angle_min
        self.scan_angle_increment = msg.angle_increment
       
        self.front_min = self.get_clearance_slice(0.0, 15.0) # Monitora a menor distância à frente do robô

    def get_clearance_slice(self, center_deg, half_width_deg):
        if self.scan_ranges is None:
            return float('inf')
        best = float('inf')
        two_pi = 2 * math.pi
        deg = -half_width_deg
        while deg <= half_width_deg:
            a = math.radians(center_deg + deg)
            a_norm = (a - self.scan_angle_min) % two_pi + self.scan_angle_min
            i = int(round((a_norm - self.scan_angle_min) / self.scan_angle_increment)) % self.scan_n
            r = self.scan_ranges[i]
            if r > 0.01 and not math.isinf(r) and not math.isnan(r):
                if r < best:
                    best = r
            deg += 1.0
        return best

    def current_goal(self):
        if self.going_home:
            return HOME
        return TARGETS[self.target_idx][1], TARGETS[self.target_idx][2]

 
    # Verificação de Colisões usando o Laser Local
    
    def is_segment_colliding(self, x1, y1, x2, y2):
    
        if self.scan_ranges is None:
            return False

        gx, gy = self.current_goal()
        is_final_goal = (math.hypot(x2 - gx, y2 - gy) < 0.05)
        current_radius = 0.05 if is_final_goal else self.ROBOT_RADIUS

        # mostra pontos intermediários ao longo do segmento planejado
        steps = int(math.hypot(x2 - x1, y2 - y1) / 0.05) + 1
        for s in range(steps):
            t = s / float(steps)
            px = x1 + t * (x2 - x1)
            py = y1 + t * (y2 - y1)

        #Transforma o ponto global mostrado em coordenadas locais do robô
            dx = px - self.x
            dy = py - self.y
            local_dist = math.hypot(dx, dy)
            local_angle = math.atan2(dy, dx) - self.yaw
            local_angle = math.atan2(math.sin(local_angle), math.cos(local_angle))

            # Procura o índice correto no buffer do Laser
            two_pi = 2 * math.pi
            a_norm = (local_angle - self.scan_angle_min) % two_pi + self.scan_angle_min
            idx = int(round((a_norm - self.scan_angle_min) / self.scan_angle_increment)) % self.scan_n
            
            laser_r = self.scan_ranges[idx]
            if laser_r > 0.01 and not math.isinf(laser_r) and not math.isnan(laser_r):
                if local_dist >= laser_r - current_radius and local_dist <= laser_r + 0.1:
                    return True
        return False

    
    # Algoritmo RRT
    
    def plan_rrt(self, start_x, start_y, goal_x, goal_y):
        self.get_logger().info(f'RRT: Planejando caminho global de ({start_x:.2f}, {start_y:.2f}) até ({goal_x:.2f}, {goal_y:.2f})...')
        nodes = [RRTNode(start_x, start_y)]
        
        for _ in range(self.RRT_MAX_ITER):
            # Viés do Alvo: 15% de chance de sortear o próprio alvo para acelerar a convergência
            if random.random() < 0.15:
                rnd_x, rnd_y = goal_x, goal_y
            else:
                rnd_x = random.uniform(self.MAP_BOUNDS[0], self.MAP_BOUNDS[1])
                rnd_y = random.uniform(self.MAP_BOUNDS[0], self.MAP_BOUNDS[1])

            # Encontra o nó mais próximo existente na árvore
            nearest_node = min(nodes, key=lambda n: math.hypot(rnd_x - n.x, rnd_y - n.y))
            
            # Dá um passo na direção do ponto sorteado
            angle = math.atan2(rnd_y - nearest_node.y, rnd_x - nearest_node.x)
            new_x = nearest_node.x + self.RRT_STEP_SIZE * math.cos(angle)
            new_y = nearest_node.y + self.RRT_STEP_SIZE * math.sin(angle)

            # Se a expansão estiver livre de obstáculos, valida o novo nó
            if not self.is_segment_colliding(nearest_node.x, nearest_node.y, new_x, new_y):
                new_node = RRTNode(new_x, new_y)
                new_node.parent = nearest_node
                nodes.append(new_node)

                # Se chega perto o suficiente do destino final, a rota está pronta
                if math.hypot(new_x - goal_x, new_y - goal_y) < self.RRT_STEP_SIZE:
                    if not self.is_segment_colliding(new_x, new_y, goal_x, goal_y):
                        goal_node = RRTNode(goal_x, goal_y)
                        goal_node.parent = new_node
                        nodes.append(goal_node)
                        
                        # Reconstrói o caminho de trás para frente
                        raw_path = []
                        curr = goal_node
                        while curr is not None:
                            raw_path.append((curr.x, curr.y))
                            curr = curr.parent
                        raw_path.reverse()
                        
                        self.get_logger().info(f'RRT: Rota encontrada com {len(raw_path)} nós.')
                        return raw_path
                        
        self.get_logger().warn('RRT: Limite de iterações atingido sem solução direta. Tentando expansão relaxada.')
        return [(start_x, start_y), (goal_x, goal_y)]

   
    # Controle de Movimento Local
    
    def handle_following_path(self, cmd):
        if self.current_wp_idx >= len(self.path):
            self.on_goal_reached()
            return

        wp_x, wp_y = self.path[self.current_wp_idx]
        dx = wp_x - self.x
        dy = wp_y - self.y
        dist = math.hypot(dx, dy)

        # Se alcançou o waypoint intermediário corrente, avança para o próximo
        if dist < self.ARRIVAL_THRESHOLD:
            self.current_wp_idx += 1
            return

        desired_yaw = math.atan2(dy, dx)
        angle_error = desired_yaw - self.yaw
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Controle Proporcional Clássico
        if abs(angle_error) > self.YAW_ALIGN_THRESHOLD:
            # Desalinhado: rotaciona no próprio eixo
            cmd.linear.x = 0.0
            cmd.angular.z = max(-self.MAX_ANGULAR_SPEED, min(self.MAX_ANGULAR_SPEED, self.kp_angular * angle_error))
        else:
            # Alinhado: avança corrigindo o rumo
            cmd.linear.x = min(self.MAX_LINEAR_SPEED, self.kp_linear * dist)
            cmd.angular.z = self.kp_angular * angle_error

        self.cmd_pub.publish(cmd)

    def on_goal_reached(self):
        if self.going_home:
            self.get_logger().info('Robô retornou à posição HOME.')
            self.going_home = False
            self.target_idx += 1
            if self.target_idx >= len(TARGETS):
                self.get_logger().info('Missão 100% Concluída com Sucesso!')
                self.state = DONE
                self.timer.cancel()
                return
        else:
            name = TARGETS[self.target_idx][0]
            self.get_logger().info(f'Alvo "{name}" alcançado!')
            self.going_home = True
        
        self.state = PLANNING

   
    # Loop de Controle Principal (FSM)

    def control_loop(self):
        if self.scan_ranges is None:
            return

        cmd = Twist()
        now = self.get_clock().now()

        # Gatilho de segurança em tempo real (Modo RECOVERING imediato se algo estiver colidindo)
        if self.state != RECOVERING and self.front_min < self.EMERGENCY_DISTANCE:
            self.state = RECOVERING
            self.recover_start_time = now

        # Máquina de Estados
        if self.state == PLANNING:
            gx, gy = self.current_goal()
            self.path = self.plan_rrt(self.x, self.y, gx, gy)
            self.current_wp_idx = 0
            self.state = FOLLOWING_PATH

        elif self.state == FOLLOWING_PATH:
            self.handle_following_path(cmd)

        elif self.state == RECOVERING:
            elapsed = (now - self.recover_start_time).nanoseconds / 1e9
            if elapsed >= self.MAX_BACKUP_DURATION:
                self.get_logger().info('Recuo concluído. Replanejando rota global...')
                self.state = PLANNING
                self.recover_start_time = None
            else:
                cmd.linear.x = -self.BACKUP_SPEED
                cmd.angular.z = 0.0
                self.cmd_pub.publish(cmd)

        elif self.state == DONE:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
