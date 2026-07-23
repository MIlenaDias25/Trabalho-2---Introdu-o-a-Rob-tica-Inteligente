# Navegação Autônoma com RRT — `robot_navigation`

Este pacote ROS 2 implementa um nó de navegação (`navigator_node`) que guia o robô por uma sequência de quatro pontos coloridos, retornando à posição inicial (`HOME`) entre cada um. O planejamento de rota usa **RRT** (Rapidly-exploring Random Tree) para encontrar um caminho global até o alvo, e um controlador proporcional local para seguir os waypoints gerados, com um modo de recuo de emergência como rede de segurança contra colisões.

## Pré-requisitos

- ROS 2 (testado no Humble)
- Ambiente de simulação já configurado (Gazebo + mundo com o robô, LIDAR e os alvos coloridos)
- Python 3.10+

## 1. Baixe o repositório do Bryan primeiro

Antes de qualquer coisa, **clone o repositório do Bryan**, que contém a base do workspace/simulação necessária para este nó funcionar:

```bash
git clone <URL_DO_REPOSITORIO_DO_BRYAN>
cd <nome-da-pasta-clonada>
```

> ⚠️ Substitua `<URL_DO_REPOSITORIO_DO_BRYAN>` pelo link real do repositório antes de compartilhar este documento.

Siga as instruções de setup desse repositório (dependências, mundo do Gazebo, etc.) antes de prosseguir.

## 2. Adicione este pacote ao workspace

Copie (ou clone) a pasta `robot_navigation` para dentro de `src/` do seu workspace ROS 2:

```bash
cd ~/ros_ws/src
# copie a pasta robot_navigation para cá
```

## 3. Compile o workspace

```bash
cd ~/ros_ws
colcon build --packages-select robot_navigation
source install/setup.bash
```

## 4. Rode a simulação

Em um terminal, suba o ambiente de simulação (Gazebo + robô), conforme as instruções do repositório do Bryan:

```bash
ros2 launch <pacote_da_simulacao> <arquivo_launch>.py
```

## 5. Rode o nó de navegação

Em outro terminal (com o workspace já com `source install/setup.bash` feito):

```bash
ros2 run robot_navigation navigator
```

O robô deve planejar uma rota e começar a se mover automaticamente em direção ao primeiro alvo.

## O que o nó faz

- **Tópicos assinados:** `/odom` (posição/orientação do robô), `/scan` (LIDAR)
- **Tópico publicado:** `/cmd_vel` (comandos de velocidade)
- **Missão:** visita os 4 alvos coloridos (verde, vermelho, azul, laranja) na ordem definida em `TARGETS`, voltando para `HOME` entre cada um.
- **Planejamento (RRT):** a cada novo objetivo, gera uma árvore aleatória a partir da posição atual do robô, com 15% de viés de amostragem direto no alvo (pra acelerar a convergência), até encontrar um caminho até o destino ou atingir o limite de iterações.
- **Checagem de colisão:** cada segmento candidato do RRT é validado contra a leitura *atual* do LIDAR (`is_segment_colliding`), amostrando pontos ao longo do segmento e comparando com a distância detectada no ângulo correspondente.
- **Seguimento de caminho:** um controlador proporcional (`handle_following_path`) guia o robô waypoint a waypoint pela rota planejada — gira no lugar se o erro angular for grande, senão avança corrigindo o rumo.
- **Segurança:** modo de recuo de emergência (`RECOVERING`), acionado sempre que algo fica perigosamente perto na frente do robô; ao final do recuo, força um replanejamento (`PLANNING`) com dados de sensor atualizados.

## Acompanhando a execução

O nó publica logs úteis para acompanhar o que está acontecendo:

```bash
ros2 run robot_navigation navigator
```

Mensagens típicas:
- `RRT: Planejando caminho global de (...) até (...)...`
- `RRT: Rota encontrada com N nós.`
- `RRT: Limite de iterações atingido sem solução direta. Tentando expansão relaxada.`
- `Alvo "<cor>" alcançado!`
- `Robô retornou à posição HOME.`
- `Recuo concluído. Replanejando rota global...`
- `Missão 100% Concluída com Sucesso!`

## Parâmetros ajustáveis

Os parâmetros principais estão no `__init__` de `NavigatorNode`, em `navigator_node.py`.

**Controle cinemático:**

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `ARRIVAL_THRESHOLD` | 0.35 m | Distância considerada "chegou" a um waypoint. |
| `YAW_ALIGN_THRESHOLD` | 0.25 rad | Erro angular acima do qual o robô só gira (não anda). |
| `MAX_LINEAR_SPEED` | 0.16 m/s | Velocidade linear máxima. |
| `MAX_ANGULAR_SPEED` | 1.0 rad/s | Velocidade angular máxima. |
| `kp_linear` / `kp_angular` | 0.5 / 1.2 | Ganhos do controlador proporcional. |

**RRT:**

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `ROBOT_RADIUS` | 0.12 m | Raio usado na checagem de colisão. Ajuste conforme o robô real. |
| `RRT_STEP_SIZE` | 0.25 m | Tamanho do passo de expansão da árvore. |
| `RRT_MAX_ITER` | 3000 | Limite de iterações do planejamento antes de desistir. |
| `MAP_BOUNDS` | (-2.5, 2.5) | Limites aproximados do cenário (X e Y), usados para amostragem aleatória. |

**Recuo de emergência:**

| Parâmetro | Padrão | Descrição |
|---|---|---|
| `EMERGENCY_DISTANCE` | 0.22 m | Distância frontal que aciona o recuo. |
| `BACKUP_SPEED` | 0.08 m/s | Velocidade de recuo. |
| `MAX_BACKUP_DURATION` | 2.0 s | Duração do recuo antes de replanejar. |

Os alvos e a posição de `HOME` são definidos no topo do arquivo:

```python
TARGETS = [
    ("verde",    2.20,  2.20),
    ("vermelho", 2.15, -2.15),
    ("azul",    -2.16, -2.16),
    ("laranja", -2.00,  1.20),
]
HOME = (-2.0, 2.0)
```

## Estrutura do algoritmo (Máquina de Estados)

| Estado | Comportamento |
|---|---|
| `PLANNING` | Executa o RRT a partir da posição atual até o objetivo corrente, gera a lista de waypoints. |
| `FOLLOWING_PATH` | Segue os waypoints do caminho planejado com controle proporcional. |
| `RECOVERING` | Recuo de emergência (rede de segurança contra colisão); ao terminar, força um novo `PLANNING`. |
| `DONE` | Missão concluída. |

## ⚠️ Problema conhecido (importante): checagem de colisão sem mapa persistente

A checagem de colisão do RRT (`is_segment_colliding`) usa **apenas a leitura instantânea do LIDAR** (`self.scan_ranges`) no momento do planejamento — não existe um mapa/grade de ocupação acumulado ao longo do tempo.

Isso é um problema porque o RRT amostra pontos em **todo o `MAP_BOUNDS`** (uma área de 5×5 m no padrão atual), mas o robô só consegue validar contra obstáculos que estão em **linha de visada direta da sua posição atual**. Qualquer coisa atrás de uma parede, de uma divisória interna ou da caixa/cone — que o robô ainda não está "vendo" no instante do planejamento — é tratada como espaço livre por padrão, mesmo que na realidade haja um obstáculo ali.

**Consequência esperada:** em mapas com múltiplas divisórias internas (como o cenário usado neste projeto), o RRT pode planejar rotas que atravessam paredes fora do campo de visão do LIDAR no momento do planejamento. O robô só vai perceber o erro ao se aproximar o suficiente pra realmente detectar o obstáculo, momento em que o `RECOVERING` assume como rede de segurança e força um replanejamento — mas o novo scan ainda pode ser parcial, então o mesmo tipo de erro pode se repetir em outras partes ocultas do mapa.

**Correções possíveis (não implementadas ainda):**
1. **Grade de ocupação acumulada:** manter um grid que vai marcando células como livres/ocupadas/desconhecidas conforme novas leituras de `/scan` chegam, e checar colisão do RRT contra esse grid em vez do scan bruto — a solução padrão pra esse tipo de arquitetura.
2. **RRT de horizonte limitado:** restringir `MAP_BOUNDS` a um raio pequeno ao redor do robô e replanejar com mais frequência, reduzindo a chance de planejar através de áreas nunca vistas.
3. **Tratar "desconhecido" como obstáculo (conservador):** em vez de assumir livre por padrão, exigir confirmação explícita de espaço livre antes de aceitar um segmento — mais seguro, porém mais lento pra explorar.

Até uma dessas correções ser aplicada, espere que o robô precise de recuos/replanejamentos ocasionais em mapas com obstáculos ocultos do ponto de planejamento inicial.
