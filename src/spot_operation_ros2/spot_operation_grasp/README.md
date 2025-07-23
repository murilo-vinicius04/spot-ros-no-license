# Spot Operation Grasp

Pacote de estratégias de grasp do sistema Spot Operation ROS2, responsável pelo gerenciamento de múltiplas estratégias de grasp, execução de operações de agarrar objetos e integração com detecção de objetos.

## 📋 Visão Geral

O `spot_operation_grasp` fornece capacidades de grasp para o Spot:
- Múltiplas estratégias de grasp
- Integração com detecção YOLO
- Sistema de fallback automático
- Serviços ROS2 para execução
- Action server para operações assíncronas

## 🏗️ Arquitetura

```
spot_operation_grasp/
├── spot_operation_grasp/
│   ├── __init__.py
│   └── grasp_manager.py
├── config/
│   └── grasp_config.yaml
├── launch/
│   └── grasp.launch.py
├── strategies/
│   ├── yolo_grasp_strategy.py
│   ├── manual_grasp_strategy.py
│   └── fallback_grasp_strategy.py
├── package.xml
├── setup.py
└── README.md
```

## 🚀 Funcionalidades

### **Grasp Manager**
- Gerenciamento de múltiplas estratégias
- Sistema de fallback automático
- Estatísticas de sucesso
- Integração com detecções 3D
- Interface ROS2

### **YOLO Grasp Strategy**
- Grasp baseado em detecção YOLO
- Múltiplas fontes de imagem
- Filtragem de objetos permitidos
- Priorização por área e classe
- Integração com profundidade

### **Manual Grasp Strategy**
- Grasp manual com pose específica
- Configuração de poses predefinidas
- Validação de workspace
- Integração com MoveIt
- Feedback de execução

### **Fallback Grasp Strategy**
- Estratégia de último recurso
- Movimento seguro
- Detecção de falhas
- Recuperação automática
- Logging detalhado

## ⚙️ Configuração

### Arquivo de Configuração
```yaml
spot_operation_grasp:
  ros__parameters:
    # Configurações gerais
    general:
      default_strategy: "front_yolo"
      fallback_strategy: "side_yolo"
      timeout: 10.0
      retry_attempts: 3
    
    # Configurações YOLO
    yolo:
      model_path: "/path/to/yolo11n.pt"
      allowed_objects_csv: "/path/to/allowed_objects.csv"
      confidence_threshold: 0.5
      max_objects: 10
    
    # Configurações de estratégias
    strategies:
      front_yolo:
        image_source: "hand_color_image"
        enabled: true
        priority: 1
      
      side_yolo:
        image_source: "frontleft_fisheye_image"
        enabled: true
        priority: 2
      
      manual:
        enabled: true
        priority: 3
        poses:
          - name: "home"
            position: [0.5, 0.0, 0.3]
            orientation: [0.0, 0.0, 0.0, 1.0]
    
    # Configurações de execução
    execution:
      max_velocity: 0.5
      max_acceleration: 0.3
      position_tolerance: 0.01
      orientation_tolerance: 0.1
      enable_collision_checking: true
```

### Parâmetros Principais

| Parâmetro | Descrição | Padrão |
|-----------|-----------|--------|
| `general.default_strategy` | Estratégia padrão | "front_yolo" |
| `general.fallback_strategy` | Estratégia de fallback | "side_yolo" |
| `yolo.confidence_threshold` | Limiar de confiança YOLO | 0.5 |
| `execution.max_velocity` | Velocidade máxima | 0.5 m/s |
| `execution.position_tolerance` | Tolerância de posição | 0.01m |

## 📦 Dependências

### Dependências ROS2
- `rclpy` - Python client library
- `geometry_msgs` - Mensagens de geometria
- `sensor_msgs` - Mensagens de sensores
- `spot_operation_msgs` - Mensagens customizadas
- `action_msgs` - Mensagens de actions

### Dependências Python
- `ultralytics` - YOLO
- `opencv-python` - OpenCV
- `numpy` - Computação numérica
- `pandas` - Manipulação de dados
- `spot-ros2` - Spot Driver ROS2

## 🔧 Instalação

### Pré-requisitos
1. ROS2 Humble instalado
2. YOLO configurado
3. Spot Driver ROS2 configurado
4. Modelos treinados disponíveis

### Build
```bash
cd ~/spot_ws
colcon build --packages-select spot_operation_grasp
source install/setup.bash
```

### Instalação de Dependências
```bash
pip install ultralytics
pip install opencv-python
pip install pandas
```

## 🚀 Uso

### Launch Individual
```bash
ros2 launch spot_operation_grasp grasp.launch.py
```

### Como Módulo
```bash
# Incluído automaticamente nos launch files principais
ros2 launch spot_operation_launch spot_operation_full.launch.py
```

### Parâmetros Customizados
```bash
ros2 launch spot_operation_grasp grasp.launch.py \
  default_strategy:=manual \
  timeout:=15.0 \
  confidence_threshold:=0.7
```

## 📡 Tópicos

### Publicados
- `/grasp_results` - Resultados de grasp
- `/grasp_debug` - Informações de debug
- `/grasp_statistics` - Estatísticas de sucesso
- `/grasp_visualization` - Visualização de grasp

### Assinados
- `/object_detections_3d` - Detecções 3D de objetos
- `/camera/image_raw` - Imagem da câmera
- `/camera/depth` - Imagem de profundidade

## 🔧 Serviços

### Oferecidos
- `/execute_grasp` - Executar grasp
- `/set_grasp_strategy` - Configurar estratégia
- `/get_grasp_statistics` - Obter estatísticas
- `/reset_grasp_manager` - Resetar gerenciador

### Clientes
- `/robot_command` - Comando para o robô
- `/manipulation_api` - API de manipulação

## 🎯 Actions

### Oferecidos
- `/grasp_object` - Grasp de objeto
- `/manipulate_object` - Manipular objeto

## 🛠️ Desenvolvimento

### Estrutura de Código
```python
# Exemplo de uso do Grasp Manager
from spot_operation_grasp.grasp_manager import GraspManager

manager = GraspManager()
manager.register_strategy("custom", CustomStrategy())
result = manager.execute_grasp("custom", robot_manager)
```

### Adicionando Novas Estratégias
```python
# Exemplo de estratégia customizada
class CustomGraspStrategy(GraspStrategyBase):
    def __init__(self):
        self.name = "custom_strategy"
    
    def execute(self, robot_manager):
        # Implementar lógica de grasp
        return success
    
    def validate(self, detection):
        # Validar se estratégia é aplicável
        return valid
```

### Integração com YOLO
```python
# Exemplo de integração YOLO
from ultralytics import YOLO
import cv2

model = YOLO("yolo11n.pt")
image = cv2.imread("test.jpg")
results = model(image)

for result in results:
    boxes = result.boxes.xyxy.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy()
    # Processar detecções...
```

## 🧪 Testes

### Testes Unitários
```bash
cd ~/spot_ws
colcon test --packages-select spot_operation_grasp
```

### Testes de Grasp
```bash
# Testar grasp com objeto simulado
ros2 run spot_operation_grasp test_grasp --object_type "cup" --position "0.5, 0.0, 0.3"
```

### Testes de Integração
```bash
# Testar com detecção real
ros2 launch spot_operation_grasp grasp.launch.py enable_camera:=true
```

## 🐛 Troubleshooting

### Problemas Comuns

#### 1. Erro de Detecção
```
Erro: Nenhum objeto detectado
```
**Solução:**
- Verificar iluminação
- Ajustar limiar de confiança
- Verificar modelo YOLO
- Calibrar câmera

#### 2. Erro de Execução
```
Erro: Falha na execução do grasp
```
**Solução:**
- Verificar workspace do braço
- Verificar colisões
- Ajustar tolerâncias
- Verificar estado do robô

#### 3. Erro de Fallback
```
Erro: Todas as estratégias falharam
```
**Solução:**
- Verificar configuração
- Verificar conectividade
- Verificar sensores
- Reiniciar sistema

### Logs de Debug
```bash
# Habilitar logs detalhados
ros2 launch spot_operation_grasp grasp.launch.py debug_mode:=true

# Monitorar resultados
ros2 topic echo /grasp_results
ros2 topic echo /grasp_debug
```

## 📊 Performance

### Métricas Típicas
- **Taxa de sucesso**: 70-90%
- **Tempo de execução**: 5-15 segundos
- **Precisão de posição**: ±2cm
- **Taxa de detecção**: 2-5 Hz

### Otimizações
- Cache de detecções
- Paralelização de estratégias
- Redução de resolução
- Uso de GPU para YOLO

## 📚 Documentação Adicional

- [YOLO Documentation](https://docs.ultralytics.com/)
- [OpenCV Python Tutorial](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
- [Spot SDK Manipulation](https://dev.bostondynamics.com/docs/concepts/arm/manipulation)
- [ROS2 Actions Tutorial](https://docs.ros.org/en/humble/Tutorials/Actions.html)

## 🔄 Referência do Código Original

### Código Original: GraspStrategyBase
```python
class GraspStrategyBase(ABC):
    """Interface base para estratégias de grasp."""
    
    @abstractmethod
    def execute(self, robot_manager):
        """
        Executa a estratégia de grasp.
        
        Args:
            robot_manager: Gerenciador de clientes do robô
            
        Returns:
            bool: True se o grasp foi bem-sucedido, False caso contrário
        """
        pass
```

### Código Original: YOLOGraspStrategy
```python
class YOLOGraspStrategy(GraspStrategyBase):
    """Estratégia de grasp baseada em detecção YOLO."""
    
    def __init__(self, detector, image_source="hand_color_image"):
        self.detector = detector
        self.image_source = image_source
        
    def execute(self, robot_manager):
        """Executa grasp com base na detecção YOLO."""
        # Captura imagem do robô
        resp = robot_manager.image_client.get_image_from_sources([self.image_source])[0]
        arr = np.frombuffer(resp.shot.image.data, dtype=np.uint8)
        image = (arr.reshape(resp.shot.image.rows, resp.shot.image.cols) 
               if resp.shot.image.format == image_pb2.Image.FORMAT_RAW
               else cv2.imdecode(arr, -1))
            
        # Detecta e filtra objetos
        boxes, _, names = self.detector.detect_objects(image)
        candidates = self.detector.filter_allowed_objects(boxes, names)
        
        if not candidates:
            rospy.logwarn("[GRASP] Nenhum objeto permitido detectado na imagem.")
            return False
            
        # Seleciona o melhor candidato
        _, _, selected_idx = candidates[0]
        x1, y1, x2, y2 = boxes[selected_idx]
        cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
        
        # Executa grasp na posição determinada
        pick = manipulation_api_pb2.PickObjectInImage(
            pixel_xy=geometry_pb2.Vec2(x=cx, y=cy),
            transforms_snapshot_for_camera=resp.shot.transforms_snapshot,
            frame_name_image_sensor=resp.shot.frame_name_image_sensor,
            camera_model=resp.source.pinhole
        )
        
        req = manipulation_api_pb2.ManipulationApiRequest(pick_object_in_image=pick)
        cmd_resp = robot_manager.manipulation_client.manipulation_api_command(req, timeout=5.0)
        
        # Monitora feedback
        result = self._monitor_grasp_feedback(robot_manager, cmd_resp)
        return result
        
    def _monitor_grasp_feedback(self, robot_manager, cmd_resp, poll_interval=0.2):
        """Monitora o feedback do comando de grasp."""
        while True:
            fb_req = manipulation_api_pb2.ManipulationApiFeedbackRequest(
                manipulation_cmd_id=cmd_resp.manipulation_cmd_id
            )
            fb = robot_manager.manipulation_client.manipulation_api_feedback_command(fb_req)
            state = manipulation_api_pb2.ManipulationFeedbackState.Name(fb.current_state)
            rospy.loginfo(f"[GRASP] Estado = {state}")
            
            if fb.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_SUCCEEDED:
                return True
            elif fb.current_state == manipulation_api_pb2.MANIP_STATE_GRASP_FAILED:
                return False
                
            rospy.sleep(poll_interval)
```

### Código Original: GraspManager
```python
class GraspManager:
    """Gerencia diferentes estratégias de grasp."""
    
    def __init__(self):
        self.strategies = {}
        self.last_used_strategy = None
        self.success_count = {}
        self.attempt_count = {}
        
    def register_strategy(self, name, strategy):
        """Registra uma nova estratégia de grasp."""
        self.strategies[name] = strategy
        self.success_count[name] = 0
        self.attempt_count[name] = 0
        
    def execute_grasp(self, strategy_name, robot_manager):
        """
        Executa a estratégia de grasp especificada.
        
        Args:
            strategy_name: Nome da estratégia a ser executada
            robot_manager: Gerenciador de clientes do robô
            
        Returns:
            bool: True se o grasp foi bem-sucedido, False caso contrário
        """
        if strategy_name not in self.strategies:
            rospy.logerr(f"Estratégia '{strategy_name}' não encontrada.")
            return False
            
        strategy = self.strategies[strategy_name]
        self.last_used_strategy = strategy_name
        self.attempt_count[strategy_name] += 1
        
        # Executa a estratégia e registra resultado
        result = strategy.execute(robot_manager)
        if result:
            self.success_count[strategy_name] += 1
            
        # Loga estatísticas de sucesso
        success_rate = self.success_count[strategy_name] / self.attempt_count[strategy_name] * 100
        rospy.loginfo(f"Estratégia '{strategy_name}' - Taxa de sucesso: {success_rate:.1f}% ({self.success_count[strategy_name]}/{self.attempt_count[strategy_name]})")
        
        return result
        
    def get_success_rates(self):
        """Retorna as taxas de sucesso de todas as estratégias registradas."""
        rates = {}
        for name in self.strategies:
            if self.attempt_count[name] > 0:
                rates[name] = self.success_count[name] / self.attempt_count[name]
            else:
                rates[name] = 0.0
        return rates
```

### Código Original: ObjectDetector
```python
class ObjectDetector:
    """Gerencia detecção de objetos usando YOLO."""
    
    def __init__(self, model_path, allowed_objects_csv):
        """
        Inicializa detector com modelo YOLO e CSV de objetos permitidos.
        
        Args:
            model_path: Caminho para o arquivo do modelo YOLO
            allowed_objects_csv: Caminho para CSV com objetos permitidos e prioridades
        """
        self.model = YOLO(model_path)
        self.allowed_objects = pd.read_csv(allowed_objects_csv)
        self.allowed_objects['class'] = self.allowed_objects['class'].str.strip().str.lower()
        
    def detect_objects(self, image):
        """Detecta objetos na imagem e retorna informações relevantes."""
        results = self.model(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        boxes = results[0].boxes.xyxy.cpu().numpy()
        classes = results[0].boxes.cls.cpu().numpy().astype(int)
        names = [results[0].names[c].lower() for c in classes]
        
        return boxes, classes, names
        
    def filter_allowed_objects(self, boxes, names):
        """Filtra objetos detectados mantendo apenas os permitidos."""
        candidates = []
        
        for i, name in enumerate(names):
            df = self.allowed_objects[self.allowed_objects['class'] == name]
            if df.empty:
                continue
                
            priority = int(df['priority'].iloc[0])
            x1, y1, x2, y2 = boxes[i]
            area = (x2 - x1) * (y2 - y1)
            
            # Armazena (prioridade, -área, índice) para ordenação
            candidates.append((priority, -area, i))
            
        # Ordena por prioridade (menor é melhor) e depois por área (maior é melhor)
        candidates.sort()
        return candidates
```

### Código Original: Integração no SpotController
```python
# Inicializa gerenciador de grasp
self.grasp_manager = GraspManager()

# Registra múltiplas estratégias de grasp com diferentes fontes de imagem
# Câmera principal (frontal)
front_grasp = YOLOGraspStrategy(
    detector=self.object_detector,
    image_source="hand_color_image"
)
self.grasp_manager.register_strategy("front_yolo", front_grasp)

# Câmera lateral (se disponível)
side_grasp = YOLOGraspStrategy(
    detector=self.object_detector,
    image_source="frontleft_fisheye_image"
)
self.grasp_manager.register_strategy("side_yolo", side_grasp)

# Estratégia padrão
self.default_grasp_strategy = "front_yolo"
# Estratégia fallback (para usar quando a principal falhar)
self.fallback_grasp_strategy = "side_yolo"

# Execução de grasp no control loop:
# Tenta executar a estratégia primária
result = self.grasp_manager.execute_grasp(self.default_grasp_strategy, self.robot_manager)

# Se falhar, tenta o fallback
if not result:
    rospy.logwarn("Grasp primário falhou. Tentando estratégia alternativa...")
    result = self.grasp_manager.execute_grasp(self.fallback_grasp_strategy, self.robot_manager)
    if not result:
        rospy.logerr("Todas as estratégias de grasp falharam.")
    else:
        rospy.loginfo("Grasp alternativo concluído com sucesso!")
        self._enter_manipulation_mode()
else:
    rospy.loginfo("Grasp primário concluído com sucesso!")
    self._enter_manipulation_mode()
```

### Migração ROS1 → ROS2
- **ROS1**: `rospy.loginfo` → **ROS2**: `self.get_logger().info`
- **ROS1**: `rospy.sleep` → **ROS2**: `time.sleep`
- **ROS1**: `rospy.ServiceProxy` → **ROS2**: `create_client`
- **ROS1**: `rospy.Publisher` → **ROS2**: `create_publisher`
- **ROS1**: `rospy.Subscriber` → **ROS2**: `create_subscription`

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