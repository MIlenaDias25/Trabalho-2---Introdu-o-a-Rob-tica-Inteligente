import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/milena/trabalho2_robotica_2026/install/turtlebot3_teleop'
