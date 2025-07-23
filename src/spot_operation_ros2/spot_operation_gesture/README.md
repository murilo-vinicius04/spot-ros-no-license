# Spot Operation Gesture

Pacote de detecção e controle por gestos do sistema Spot Operation ROS2, responsável pela detecção de gestos, contagem de dedos e interface de comando para o robô Spot.

## 📋 Visão Geral

O `spot_operation_gesture` fornece capacidades de controle por gestos para o Spot:
- Detecção de gestos baseada em contagem de dedos
- Interface de serviço ROS2
- Publicação de comandos de gesto
- Integração com sistema de controle
- Configuração flexível de intervalos e tópicos

## 🏗️ Arquitetura

```
spot_operation_gesture/
├── spot_operation_gesture/
│   ├── __init__.py
│   └── finger_count_client.py
├── config/
│   └── gesture_config.yaml
├── launch/
│   └── gesture.launch.py
├── gestures/
│   ├── gesture_detector.py
│   └── gesture_mapper.py
├── package.xml
├── setup.py
└── README.md
```

## 🚀 Funcionalidades

### **Finger Count Client**
- Cliente para serviço de contagem de dedos
- Interface de serviço ROS2
- Publicação de comandos de gesto
- Configuração de intervalos
- Integração com sistema de controle

### **Gesture Detector**
- Detecção de gestos em tempo real
- Contagem de dedos
- Classificação de gestos
- Filtragem de ruído
- Calibração automática

### **Gesture Mapper**
- Mapeamento de gestos para comandos
- Configuração de comandos customizados
- Múltiplos modos de mapeamento
- Integração com sistema de controle
- Logging de comandos

## ⚙️ Configuração

### Arquivo de Configuração
```yaml
spot_operation_gesture:
  ros__parameters:
    # Configurações do serviço
    service:
      service_name: "/finger_count_node/get_finger_count"
      timeout: 1.0
      retry_attempts: 3
    
    # Configurações de detecção
    detection:
      check_interval: 0.5
      confidence_threshold: 0.7
      min_gesture_duration: 1.0
      max_gesture_duration: 10.0
    
    # Configurações de publicação
    publishing:
      publish_topic: "gesture_command"
      publish_rate: 2.0
      enable_debug: false
      publish_gesture_image: false
    
    # Configurações de mapeamento
    mapping:
      enable_gesture_mapping: true
      default_mode: "manual"
      gesture_commands:
        0: "stop"
        1: "forward"
        2: "backward"
        3: "left"
        4: "right"
        5: "grasp"
```

### Parâmetros Principais

| Parâmetro | Descrição | Padrão |
|-----------|-----------|--------|
| `service.service_name` | Nome do serviço de contagem | "/finger_count_node/get_finger_count" |
| `detection.check_interval` | Intervalo de verificação | 0.5s |
| `detection.confidence_threshold` | Limiar de confiança | 0.7 |
| `publishing.publish_topic` | Tópico de publicação | "gesture_command" |
| `mapping.enable_gesture_mapping` | Habilitar mapeamento | true |

## 📦 Dependências

### Dependências ROS2
- `rclpy` - Python client library
- `std_msgs` - Mensagens padrão
- `spot_operation_msgs` - Mensagens customizadas
- `sensor_msgs` - Mensagens de sensores

### Dependências Python
- `opencv-python` - OpenCV
- `numpy` - Computação numérica
- `mediapipe` - Detecção de mãos (opcional)
- `cvzone` - Detecção de gestos (opcional)

## 🔧 Instalação

### Pré-requisitos
1. ROS2 Humble instalado
2. OpenCV instalado
3. Câmera configurada
4. Workspace ROS2 configurado

### Build
```bash
cd ~/spot_ws
colcon build --packages-select spot_operation_gesture
source install/setup.bash
```

### Instalação de Dependências Opcionais
```bash
pip install mediapipe
pip install cvzone
```

## 🚀 Uso

### Launch Individual
```bash
ros2 launch spot_operation_gesture gesture.launch.py
```

### Como Módulo
```bash
# Incluído automaticamente nos launch files principais
ros2 launch spot_operation_launch spot_operation_full.launch.py
```

### Parâmetros Customizados
```bash
ros2 launch spot_operation_gesture gesture.launch.py \
  service_name:=/custom_finger_count \
  check_interval:=0.3 \
  publish_topic:=custom_gesture_command
```

## 📡 Tópicos

### Publicados
- `/gesture_command` - Comandos de gesto (GestureCommand)
- `/gesture_debug` - Informações de debug
- `/gesture_image` - Imagem com detecções (debug)

### Assinados
- `/camera/image_raw` - Imagem da câmera (opcional)
- `/finger_count_result` - Resultado da contagem (opcional)

## 🔧 Serviços

### Clientes
- `/finger_count_node/get_finger_count` - Obter contagem de dedos
- `/gesture_detection/get_gesture` - Obter gesto atual

### Oferecidos
- `/set_gesture_mapping` - Configurar mapeamento
- `/get_gesture_stats` - Obter estatísticas
- `/calibrate_gesture` - Calibrar detecção

## 🎯 Gestos Suportados

### Gestos de Modo de Operação
- **1 dedo**: Modo Manual
- **2 dedos**: Modo Semi-Autônomo
- **3 dedos**: Solicitar seleção de modo

### Gestos de Controle (no modo manual)
- **0 dedos**: Abrir gripper / Parar movimento
- **1 dedo**: Fechar gripper / Executar grasp

### Gestos de Manipulação
- **0 dedos**: Liberar objeto (sair do modo manipulação)
- **Outros**: Continuar no modo manipulação

## 🛠️ Desenvolvimento

### Estrutura de Código
```python
# Exemplo de uso do Finger Count Client
from spot_operation_gesture.finger_count_client import FingerCountClient

client = FingerCountClient()
client.set_service_name("/custom_finger_count")
gesture = client.get_gesture()
```

### Adicionando Novos Gestos
```python
# Exemplo de gesto customizado
class CustomGesture:
    def __init__(self):
        self.name = "custom_gesture"
        self.finger_count = 6
    
    def detect(self, image):
        # Implementar detecção
        return detected
    
    def get_command(self):
        # Retornar comando
        return "custom_command"
```

### Integração com Câmera
```python
# Exemplo de integração com câmera
import cv2
from spot_operation_gesture.gesture_detector import GestureDetector

detector = GestureDetector()
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    gesture = detector.detect(frame)
    if gesture:
        print(f"Gesto detectado: {gesture}")
```

## 🧪 Testes

### Testes Unitários
```bash
cd ~/spot_ws
colcon test --packages-select spot_operation_gesture
```

### Testes de Detecção
```bash
# Testar detecção com imagem
ros2 run spot_operation_gesture test_detection --image_path test_image.jpg
```

### Testes de Integração
```bash
# Testar com câmera
ros2 launch spot_operation_gesture gesture.launch.py enable_camera:=true
```

## 🐛 Troubleshooting

### Problemas Comuns

#### 1. Erro de Serviço
```
Erro: Serviço de contagem não encontrado
```
**Solução:**
- Verificar se serviço está rodando
- Verificar nome do serviço
- Verificar conectividade
- Reiniciar serviço

#### 2. Baixa Precisão
```
Problema: Detecção imprecisa
```
**Solução:**
- Melhorar iluminação
- Ajustar câmera
- Calibrar detecção
- Usar filtros

#### 3. Latência Alta
```
Problema: Resposta lenta
```
**Solução:**
- Reduzir intervalo de verificação
- Otimizar algoritmo
- Usar hardware mais rápido
- Reduzir resolução

### Logs de Debug
```bash
# Habilitar logs detalhados
ros2 launch spot_operation_gesture gesture.launch.py enable_debug:=true

# Monitorar comandos
ros2 topic echo /gesture_command
ros2 topic echo /gesture_debug
```

## 📊 Performance

### Métricas Típicas
- **Taxa de detecção**: 2-5 Hz
- **Precisão**: 85-95%
- **Latência**: 100-500ms
- **Taxa de falsos positivos**: <5%

### Otimizações
- Redução de resolução
- Filtros temporais
- Cache de detecções
- Uso de GPU

## 📚 Documentação Adicional

- [OpenCV Python Tutorial](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
- [MediaPipe Hands](https://google.github.io/mediapipe/solutions/hands)
- [ROS2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Services.html)
- [Computer Vision for Robotics](https://opencv.org/)

## 🔄 Referência do Código Original

### Código Original: FingerCountClient
```python
class FingerCountClient:
    """Calls /finger_count_node/get_finger_count until it gets 1 or 2."""
    def __init__(self, service_name="/finger_count_node/get_finger_count"):
        rospy.wait_for_service(service_name)
        self._proxy = rospy.ServiceProxy(service_name, Trigger)

    def request_mode(self):
        """Blocks until receiving 1 (manual) or 2 (semi-autonomous)."""
        while not rospy.is_shutdown():
            try:
                resp = self._proxy()
            except rospy.ServiceException as e:
                rospy.logwarn("FingerCount service failed: %s", e)
                rospy.sleep(0.5)
                continue

            if resp.success and resp.message in ("1", "2"):
                return int(resp.message)

            rospy.loginfo("FingerCount returned '%s'. Retrying...", resp.message)
            rospy.sleep(0.3)

    def check_and_update_mode(self, current_mode):
        """Checa uma vez se o número de dedos mudou e atualiza o modo se necessário."""
        try:
            resp = self._proxy()
            if not resp.success or resp.message not in ("1", "2", "3"):
                return current_mode

            code = int(resp.message)
            expected = 1 if current_mode == "manual" else 2
            
            if code == expected:
                return current_mode
            elif code in (1, 2):
                novo = "manual" if code == 1 else "semi"
                rospy.loginfo("🔁 Modo alterado dinamicamente para: %s", novo.upper())
                return novo
            elif code == 3:
                rospy.loginfo("👆 Gesto de MODO detectado → solicitando novo modo")
                while not rospy.is_shutdown():
                    resp = self._proxy()
                    if resp.success and resp.message in ("1", "2"):
                        novo = "manual" if resp.message == "1" else "semi"
                        rospy.loginfo("✅ Novo modo definido: %s", novo.upper())
                        return novo
                    rospy.sleep(0.5)
        except rospy.ServiceException as e:
            rospy.logwarn("Erro ao consultar modo de dedo: %s", e)
        return current_mode
```

### Código Original: Gesture Callback
```python
def _gesture_callback(self, msg):
    """Callback para mensagens de gestos."""
    self.current_gesture = msg.data

# Uso no código original:
if self.current_gesture == 0:
    self.robot_manager.open_gripper()
else:
    self.robot_manager.close_gripper()

# Verificação de grasp:
if self.current_gesture == 1:
    rospy.loginfo("✋ Gesto de agarrar detectado!")
    # Executa grasp...

# Verificação de modo manipulação:
if self.current_gesture == 0:
    self._exit_manipulation_mode()
```

### Migração ROS1 → ROS2
- **ROS1**: `rospy.ServiceProxy` → **ROS2**: `create_client`
- **ROS1**: `rospy.wait_for_service` → **ROS2**: `wait_for_service`
- **ROS1**: `rospy.loginfo` → **ROS2**: `self.get_logger().info`
- **ROS1**: `rospy.sleep` → **ROS2**: `time.sleep`
- **ROS1**: `rospy.is_shutdown` → **ROS2**: `rclpy.ok()`

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