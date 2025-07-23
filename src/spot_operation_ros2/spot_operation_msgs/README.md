# Spot Operation Messages

Pacote de mensagens customizadas do sistema Spot Operation ROS2, definindo todas as mensagens, serviços e actions específicas para operação do robô Spot.

## 📋 Visão Geral

O `spot_operation_msgs` define as interfaces de comunicação do sistema:
- Mensagens para detecção de objetos
- Mensagens para resultados de grasp
- Mensagens para comandos de gesto
- Serviços para execução de operações
- Actions para operações assíncronas

## 🏗️ Arquitetura

```
spot_operation_msgs/
├── action/
│   ├── GraspObject.action
│   └── ManipulateObject.action
├── msg/
│   ├── GestureCommand.msg
│   ├── GraspResult.msg
│   ├── ObjectDetection.msg
│   └── OperationState.msg
├── srv/
│   ├── ExecuteGrasp.srv
│   ├── GetOperationState.srv
│   └── SetGraspStrategy.srv
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 📦 Tipos de Mensagens

### **Actions**

#### GraspObject.action
```idl
# Goal
geometry_msgs/PoseStamped target_pose
string strategy
float32 timeout
---
# Result
bool success
string message
geometry_msgs/PoseStamped final_pose
---
# Feedback
float32 progress
string status
```

#### ManipulateObject.action
```idl
# Goal
geometry_msgs/PoseStamped start_pose
geometry_msgs/PoseStamped end_pose
string manipulation_type
float32 timeout
---
# Result
bool success
string message
geometry_msgs/PoseStamped final_pose
---
# Feedback
float32 progress
string status
```

### **Messages**

#### GestureCommand.msg
```idl
Header header
uint8 gesture_type
uint8 finger_count
string command
float32 confidence
bool is_active
```

#### GraspResult.msg
```idl
Header header
bool success
string strategy_used
geometry_msgs/PoseStamped object_pose
float32 grasp_quality
string message
```

#### ObjectDetection.msg
```idl
Header header
string object_class
float32 confidence
geometry_msgs/PoseStamped pose
geometry_msgs/Vector3 dimensions
uint32 detection_id
```

#### OperationState.msg
```idl
Header header
string current_mode
string current_state
bool is_active
float32 battery_level
string error_message
```

### **Services**

#### ExecuteGrasp.srv
```idl
# Request
geometry_msgs/PoseStamped target_pose
string strategy
float32 timeout
---
# Response
bool success
string message
geometry_msgs/PoseStamped final_pose
```

#### GetOperationState.srv
```idl
# Request
---
# Response
string current_mode
string current_state
bool is_active
float32 battery_level
string error_message
```

#### SetGraspStrategy.srv
```idl
# Request
string strategy_name
bool enable
---
# Response
bool success
string message
```

## 🔧 Uso

### Compilação
```bash
cd ~/spot_ws
colcon build --packages-select spot_operation_msgs
source install/setup.bash
```

### Importação em Python
```python
from spot_operation_msgs.msg import GestureCommand, ObjectDetection
from spot_operation_msgs.srv import ExecuteGrasp
from spot_operation_msgs.action import GraspObject
```

### Importação em C++
```cpp
#include "spot_operation_msgs/msg/gesture_command.hpp"
#include "spot_operation_msgs/msg/object_detection.hpp"
#include "spot_operation_msgs/srv/execute_grasp.hpp"
#include "spot_operation_msgs/action/grasp_object.hpp"
```

## 📡 Exemplos de Uso

### Publicando Mensagem
```python
import rclpy
from rclpy.node import Node
from spot_operation_msgs.msg import GestureCommand

class GesturePublisher(Node):
    def __init__(self):
        super().__init__('gesture_publisher')
        self.publisher = self.create_publisher(GestureCommand, 'gesture_command', 10)
        
    def publish_gesture(self, finger_count, command):
        msg = GestureCommand()
        msg.finger_count = finger_count
        msg.command = command
        msg.confidence = 0.9
        msg.is_active = True
        self.publisher.publish(msg)
```

### Assinando Mensagem
```python
from spot_operation_msgs.msg import ObjectDetection

class ObjectSubscriber(Node):
    def __init__(self):
        super().__init__('object_subscriber')
        self.subscription = self.create_subscription(
            ObjectDetection,
            'object_detections_3d',
            self.listener_callback,
            10)
    
    def listener_callback(self, msg):
        self.get_logger().info(f'Detected: {msg.object_class} at {msg.pose.position}')
```

### Usando Serviço
```python
from spot_operation_msgs.srv import ExecuteGrasp
from geometry_msgs.msg import PoseStamped

class GraspClient(Node):
    def __init__(self):
        super().__init__('grasp_client')
        self.client = self.create_client(ExecuteGrasp, 'execute_grasp')
        
    def execute_grasp(self, pose, strategy):
        request = ExecuteGrasp.Request()
        request.target_pose = pose
        request.strategy = strategy
        request.timeout = 30.0
        
        future = self.client.call_async(request)
        return future
```

### Usando Action
```python
from spot_operation_msgs.action import GraspObject
from rclpy.action import ActionClient

class GraspActionClient(Node):
    def __init__(self):
        super().__init__('grasp_action_client')
        self.action_client = ActionClient(self, GraspObject, 'grasp_object')
        
    def send_grasp_goal(self, pose, strategy):
        goal_msg = GraspObject.Goal()
        goal_msg.target_pose = pose
        goal_msg.strategy = strategy
        goal_msg.timeout = 30.0
        
        self.action_client.send_goal_async(goal_msg)
```

## 🛠️ Desenvolvimento

### Adicionando Nova Mensagem
1. Criar arquivo `.msg` em `msg/`
2. Definir estrutura da mensagem
3. Atualizar `CMakeLists.txt`
4. Compilar pacote
5. Testar importação

### Adicionando Novo Serviço
1. Criar arquivo `.srv` em `srv/`
2. Definir Request e Response
3. Atualizar `CMakeLists.txt`
4. Compilar pacote
5. Testar serviço

### Adicionando Nova Action
1. Criar arquivo `.action` em `action/`
2. Definir Goal, Result e Feedback
3. Atualizar `CMakeLists.txt`
4. Compilar pacote
5. Testar action

## 🧪 Testes

### Testes de Compilação
```bash
cd ~/spot_ws
colcon build --packages-select spot_operation_msgs
```

### Testes de Importação
```bash
# Python
python3 -c "from spot_operation_msgs.msg import GestureCommand; print('OK')"

# C++
ros2 run spot_operation_msgs test_import
```

### Testes de Comunicação
```bash
# Testar publicação/assinatura
ros2 topic pub /test_gesture spot_operation_msgs/msg/GestureCommand "{finger_count: 1, command: 'forward'}"

# Testar serviço
ros2 service call /test_grasp spot_operation_msgs/srv/ExecuteGrasp "{target_pose: {pose: {position: {x: 0.5, y: 0.0, z: 0.3}}}}, strategy: 'yolo_grasp'}"
```

## 📊 Convenções

### Nomenclatura
- **Mensagens**: PascalCase (ex: `GestureCommand`)
- **Campos**: snake_case (ex: `finger_count`)
- **Serviços**: PascalCase (ex: `ExecuteGrasp`)
- **Actions**: PascalCase (ex: `GraspObject`)

### Tipos de Dados
- **Identificadores**: `uint32`
- **Confiança**: `float32`
- **Posições**: `geometry_msgs/PoseStamped`
- **Dimensões**: `geometry_msgs/Vector3`
- **Comandos**: `string`
- **Estados**: `string`

### Headers
- Todas as mensagens devem incluir `Header header`
- Usar timestamp atual
- Definir frame_id apropriado

## 📚 Documentação Adicional

- [ROS2 Messages](https://docs.ros.org/en/humble/Concepts/Basic/About-ROS-Interfaces.html)
- [ROS2 Services](https://docs.ros.org/en/humble/Tutorials/Services.html)
- [ROS2 Actions](https://docs.ros.org/en/humble/Tutorials/Actions.html)
- [Message Generation](https://docs.ros.org/en/humble/Guides/Interface-definition.html)

## 🤝 Contribuição

Para contribuir com este pacote:

1. Fork do repositório
2. Criar branch para feature
3. Implementar mudanças
4. Adicionar testes
5. Submeter pull request

## 📞 Suporte

Para problemas ou dúvidas:
1. Verificar logs do sistema
2. Consultar documentação
3. Abrir issue no repositório
4. Contatar equipe de desenvolvimento 