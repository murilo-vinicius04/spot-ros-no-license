# Spot Operation Launch System

Este pacote contém os launch files para o sistema completo de operação do Spot, permitindo diferentes modos de operação e configurações.

## 🚀 Launch Files Disponíveis

### 1. **spot_operation_full.launch.py** - Sistema Completo
Lança todos os módulos do sistema de operação do Spot.

```bash
# Lançar sistema completo
ros2 launch spot_operation_launch spot_operation_full.launch.py

# Lançar sem RViz
ros2 launch spot_operation_launch spot_operation_full.launch.py launch_rviz:=false

# Lançar apenas módulos específicos
ros2 launch spot_operation_launch spot_operation_full.launch.py \
  launch_vision:=true \
  launch_grasp:=true \
  launch_control:=true \
  launch_gesture:=false
```

### 2. **spot_operation_manual.launch.py** - Modo Manual
Focado em controle manual via gestos, ideal para teleoperação.

```bash
# Modo manual básico
ros2 launch spot_operation_launch spot_operation_manual.launch.py

# Modo manual com visão habilitada
ros2 launch spot_operation_launch spot_operation_manual.launch.py enable_vision:=true

# Modo manual com grasp habilitado
ros2 launch spot_operation_launch spot_operation_manual.launch.py enable_grasp:=true
```

### 3. **spot_operation_autonomous.launch.py** - Modo Autônomo
Sistema autônomo com detecção e manipulação automática de objetos.

```bash
# Modo autônomo completo
ros2 launch spot_operation_launch spot_operation_autonomous.launch.py

# Modo autônomo com limiar de confiança personalizado
ros2 launch spot_operation_launch spot_operation_autonomous.launch.py confidence_threshold:=0.8

# Modo autônomo sem gestos
ros2 launch spot_operation_launch spot_operation_autonomous.launch.py enable_gestures:=false
```

### 4. **spot_operation_debug.launch.py** - Modo Debug
Sistema com ferramentas de debug e análise detalhada.

```bash
# Modo debug completo
ros2 launch spot_operation_launch spot_operation_debug.launch.py

# Modo debug sem RQT
ros2 launch spot_operation_launch spot_operation_debug.launch.py enable_rqt:=false

# Modo debug com nível específico
ros2 launch spot_operation_launch spot_operation_debug.launch.py debug_level:=INFO
```

## 📋 Módulos do Sistema

### **spot_operation_core**
- Gerenciamento de estado da operação
- Cliente do robô Spot
- Configurações globais

### **spot_operation_vision**
- Detecção de objetos com YOLO
- Processamento de profundidade
- Publicação de detecções 3D

### **spot_operation_grasp**
- Estratégias de grasp
- Serviços de execução
- Resultados de manipulação

### **spot_operation_control**
- Controle do braço com MoveIt
- Campos de potencial
- Modos de operação

### **spot_operation_gesture**
- Detecção de gestos
- Controle por gestos
- Interface de comando

## ⚙️ Parâmetros Configuráveis

### Parâmetros Globais
- `launch_core`: Habilitar módulo core (padrão: true)
- `launch_vision`: Habilitar módulo de visão (padrão: true)
- `launch_grasp`: Habilitar módulo de grasp (padrão: true)
- `launch_control`: Habilitar módulo de controle (padrão: true)
- `launch_gesture`: Habilitar módulo de gestos (padrão: true)
- `launch_rviz`: Habilitar RViz (padrão: true)

### Parâmetros de Visão
- `confidence_threshold`: Limiar de confiança para detecção (padrão: 0.7)
- `debug_mode`: Modo debug para visão (padrão: false)
- `publish_debug_images`: Publicar imagens de debug (padrão: false)

### Parâmetros de Grasp
- `default_strategy`: Estratégia de grasp padrão (padrão: 'yolo_grasp')
- `fallback_strategy`: Estratégia de fallback (padrão: 'manual_grasp')
- `debug_mode`: Modo debug para grasp (padrão: false)

### Parâmetros de Controle
- `group_name`: Nome do grupo MoveIt (padrão: 'manipulator')
- `end_effector_link`: Link do end-effector (padrão: 'arm_link_fngr')
- `control_rate`: Taxa de controle em Hz (padrão: 5.0)
- `pf_k_att`: Ganho de atração do campo de potencial (padrão: 0.2)
- `pf_attraction_distance`: Distância de atração (padrão: 0.4)
- `escape_distance`: Distância de escape (padrão: 0.2)
- `debug_mode`: Modo debug para controle (padrão: false)

### Parâmetros de Gestos
- `service_name`: Nome do serviço de contagem de dedos
- `check_interval`: Intervalo de verificação em segundos (padrão: 0.5)
- `publish_topic`: Tópico de publicação de comandos (padrão: 'gesture_command')
- `debug_mode`: Modo debug para gestos (padrão: false)

## 🛠️ Ferramentas de Debug

### RViz
- Visualização do robô e ambiente
- Marcadores de detecções 3D
- Resultados de grasp
- Trajetórias planejadas

### RQT
- Interface gráfica para debug
- Monitoramento de tópicos
- Configuração de parâmetros
- Análise de dados

### PlotJuggler
- Visualização de dados temporais
- Análise de sinais
- Debug de algoritmos

## 📁 Estrutura de Arquivos

```
spot_operation_launch/
├── launch/
│   ├── spot_operation_full.launch.py
│   ├── spot_operation_manual.launch.py
│   ├── spot_operation_autonomous.launch.py
│   └── spot_operation_debug.launch.py
├── config/
│   ├── spot_operation.rviz
│   ├── spot_operation_manual.rviz
│   ├── spot_operation_autonomous.rviz
│   ├── spot_operation_debug.rviz
│   ├── spot_operation_debug.perspective
│   └── spot_operation_debug.xml
└── README.md
```

## 🔧 Pré-requisitos

1. **ROS2 Humble** instalado
2. **Spot Driver ROS2** configurado
3. **MoveIt2** instalado
4. **YOLO** configurado para detecção
5. **RealSense** ou câmera compatível

## 🚀 Exemplos de Uso

### Início Rápido
```bash
# 1. Build do workspace
cd ~/spot_ws
colcon build --packages-select spot_operation_ros2

# 2. Source do workspace
source install/setup.bash

# 3. Lançar sistema completo
ros2 launch spot_operation_launch spot_operation_full.launch.py
```

### Modo de Desenvolvimento
```bash
# Lançar modo debug
ros2 launch spot_operation_launch spot_operation_debug.launch.py

# Em outro terminal, monitorar tópicos
ros2 topic list
ros2 topic echo /object_detections_3d
ros2 topic echo /gesture_command
```

### Teste de Módulos Individuais
```bash
# Testar apenas visão
ros2 launch spot_operation_launch spot_operation_full.launch.py \
  launch_grasp:=false \
  launch_control:=false \
  launch_gesture:=false

# Testar apenas controle manual
ros2 launch spot_operation_launch spot_operation_manual.launch.py
```

## 🐛 Troubleshooting

### Problemas Comuns

1. **Erro de TF**: Verificar se o robô está conectado e publicando TF
2. **Erro de câmera**: Verificar conexão da RealSense e drivers
3. **Erro de MoveIt**: Verificar configuração do URDF e MoveIt
4. **Erro de YOLO**: Verificar modelo e dependências do YOLO

### Logs de Debug
```bash
# Ver logs detalhados
ros2 launch spot_operation_launch spot_operation_debug.launch.py

# Monitorar logs específicos
ros2 run rqt_console rqt_console
```

## 📞 Suporte

Para problemas ou dúvidas:
1. Verificar logs do sistema
2. Consultar documentação dos módulos individuais
3. Verificar configuração do hardware
4. Contatar equipe de desenvolvimento 