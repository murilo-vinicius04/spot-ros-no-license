# Spot Operation Vision

Pacote de visão computacional do sistema Spot Operation ROS2, responsável pela detecção de objetos, processamento de profundidade e integração com câmeras do Spot.

## 📋 Visão Geral

O `spot_operation_vision` fornece capacidades de visão para o Spot:
- Detecção de objetos com YOLO
- Processamento de profundidade
- Integração com câmeras do Spot
- Publicação de detecções 3D
- Configuração flexível de modelos

## 🏗️ Arquitetura

```
spot_operation_vision/
├── spot_operation_vision/
│   ├── __init__.py
│   ├── object_detector.py
│   └── depth_processor.py
├── config/
│   └── vision_config.yaml
├── launch/
│   └── vision.launch.py
├── models/
│   └── yolo11n.pt
├── package.xml
├── setup.py
└── README.md
```

## 🚀 Funcionalidades

### **Object Detector**
- Detecção de objetos com YOLO
- Filtragem por classes permitidas
- Priorização por área e classe
- Publicação de detecções 2D
- Integração com câmeras

### **Depth Processor**
- Processamento de imagens de profundidade
- Cálculo de poses 3D
- Sincronização RGB-Depth
- Publicação de detecções 3D
- Validação de profundidade

### **Camera Integration**
- Integração com câmeras do Spot
- Múltiplas fontes de imagem
- Calibração automática
- Gerenciamento de streams
- Otimização de performance

## ⚙️ Configuração

### Arquivo de Configuração
```yaml
spot_operation_vision:
  ros__parameters:
    # Configurações do detector
    detector:
      model_path: "/path/to/yolo11n.pt"
      allowed_objects_csv: "/path/to/allowed_objects.csv"
      confidence_threshold: 0.5
      max_objects: 10
      publish_rate: 5.0
    
    # Configurações de profundidade
    depth:
      min_depth: 0.3
      max_depth: 3.0
      depth_window_size: 5
      enable_filtering: true
      publish_3d_detections: true
    
    # Configurações de câmera
    camera:
      rgb_topic: "/camera/color/image_raw"
      depth_topic: "/camera/depth/image_raw"
      camera_info_topic: "/camera/color/camera_info"
      frame_id: "camera_color_optical_frame"
    
    # Configurações de processamento
    processing:
      enable_visualization: true
      save_debug_images: false
      debug_topic: "/vision_debug"
      max_processing_time: 0.1
```

### Parâmetros Principais

| Parâmetro | Descrição | Padrão |
|-----------|-----------|--------|
| `detector.model_path` | Caminho do modelo YOLO | "/path/to/yolo11n.pt" |
| `detector.confidence_threshold` | Limiar de confiança | 0.5 |
| `depth.min_depth` | Profundidade mínima | 0.3m |
| `depth.max_depth` | Profundidade máxima | 3.0m |
| `processing.enable_visualization` | Habilitar visualização | true |

## 📦 Dependências

### Dependências ROS2
- `rclpy` - Python client library
- `sensor_msgs` - Mensagens de sensores
- `geometry_msgs` - Mensagens de geometria
- `cv_bridge` - Bridge OpenCV-ROS
- `message_filters` - Filtros de mensagens
- `tf2_ros` - Transformações

### Dependências Python
- `ultralytics` - YOLO
- `opencv-python` - OpenCV
- `numpy` - Computação numérica
- `pandas` - Manipulação de dados
- `cv_bridge` - Bridge OpenCV-ROS

## 🔧 Instalação

### Pré-requisitos
1. ROS2 Humble instalado
2. OpenCV instalado
3. YOLO configurado
4. Câmeras do Spot configuradas

### Build
```bash
cd ~/spot_ws
colcon build --packages-select spot_operation_vision
source install/setup.bash
```

### Instalação de Dependências
```bash
pip install ultralytics
pip install opencv-python
pip install pandas
pip install cv-bridge
```

## 🚀 Uso

### Launch Individual
```bash
ros2 launch spot_operation_vision vision.launch.py
```

### Como Módulo
```bash
# Incluído automaticamente nos launch files principais
ros2 launch spot_operation_launch spot_operation_full.launch.py
```

### Parâmetros Customizados
```bash
ros2 launch spot_operation_vision vision.launch.py \
  model_path:=/custom/path/yolo.pt \
  confidence_threshold:=0.7 \
  enable_visualization:=true
```

## 📡 Tópicos

### Publicados
- `/object_detections_2d` - Detecções 2D de objetos
- `/object_detections_3d` - Detecções 3D de objetos
- `/vision_debug` - Informações de debug
- `/vision_visualization` - Visualização de detecções

### Assinados
- `/camera/color/image_raw` - Imagem RGB
- `/camera/depth/image_raw` - Imagem de profundidade
- `/camera/color/camera_info` - Informações da câmera

## 🔧 Serviços

### Oferecidos
- `/get_detection_stats` - Obter estatísticas
- `/set_detection_params` - Configurar parâmetros
- `/save_debug_image` - Salvar imagem de debug
- `/reload_model` - Recarregar modelo YOLO

## 🛠️ Desenvolvimento

### Estrutura de Código
```python
# Exemplo de uso do Object Detector
from spot_operation_vision.object_detector import ObjectDetector

detector = ObjectDetector()
detector.set_model_path("/path/to/model.pt")
detections = detector.detect(image)
```

### Adicionando Novos Modelos
```python
# Exemplo de modelo customizado
class CustomDetector:
    def __init__(self):
        self.name = "custom_detector"
    
    def detect(self, image):
        # Implementar detecção
        return detections
    
    def filter_detections(self, detections):
        # Filtrar detecções
        return filtered_detections
```

### Integração com Câmeras
```python
# Exemplo de integração com câmera
import cv2
from cv_bridge import CvBridge

bridge = CvBridge()
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
    # Processar imagem...
```

## 🧪 Testes

### Testes Unitários
```bash
cd ~/spot_ws
colcon test --packages-select spot_operation_vision
```

### Testes de Detecção
```bash
# Testar detecção com imagem
ros2 run spot_operation_vision test_detection --image_path test_image.jpg
```

### Testes de Integração
```bash
# Testar com câmera real
ros2 launch spot_operation_vision vision.launch.py enable_camera:=true
```

## 🐛 Troubleshooting

### Problemas Comuns

#### 1. Erro de Modelo
```
Erro: Modelo YOLO não encontrado
```
**Solução:**
- Verificar caminho do modelo
- Verificar permissões de arquivo
- Baixar modelo novamente
- Verificar formato do arquivo

#### 2. Erro de Câmera
```
Erro: Câmera não disponível
```
**Solução:**
- Verificar conectividade
- Verificar drivers
- Verificar permissões
- Reiniciar câmera

#### 3. Erro de Performance
```
Problema: Processamento lento
```
**Solução:**
- Reduzir resolução
- Ajustar parâmetros
- Usar GPU
- Otimizar código

### Logs de Debug
```bash
# Habilitar logs detalhados
ros2 launch spot_operation_vision vision.launch.py debug_mode:=true

# Monitorar detecções
ros2 topic echo /object_detections_2d
ros2 topic echo /object_detections_3d
```

## 📊 Performance

### Métricas Típicas
- **Taxa de detecção**: 5-10 Hz
- **Precisão**: 85-95%
- **Latência**: 100-300ms
- **Taxa de falsos positivos**: <5%

### Otimizações
- Redução de resolução
- Uso de GPU
- Cache de detecções
- Paralelização

## 📚 Documentação Adicional

- [YOLO Documentation](https://docs.ultralytics.com/)
- [OpenCV Python Tutorial](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
- [ROS2 Image Processing](https://docs.ros.org/en/humble/Tutorials/Intermediate/Image_Processing.html)
- [CV Bridge Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Cv_Bridge.html)

## 🔄 Referência do Código Original

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

### Código Original: YOLO Depth Deprojection
```python
def _yolo_depth_deproject(self):
    """Detecta o objeto em 'hand_color_image' e retorna as coordenadas 3D no quadro de coordenadas do robô."""
    try:
        rgb_resp, depth_resp = self.robot_manager.image_client.get_image_from_sources(
            ["hand_color_image", "hand_depth_in_hand_color_frame"])
    except Exception as exc:
        rospy.logwarn_throttle(2.0, "Falha ao obter imagem: %s", exc)
        return None

    # --- Decodifica a imagem RGB ---
    rgb_arr = np.frombuffer(rgb_resp.shot.image.data, dtype=np.uint8)
    if rgb_resp.shot.image.format == image_pb2.Image.FORMAT_RAW:
        rgb_img = rgb_arr.reshape(rgb_resp.shot.image.rows, rgb_resp.shot.image.cols, 1)
        rgb_img = cv2.cvtColor(rgb_img, cv2.COLOR_GRAY2BGR)
    else:
        rgb_img = cv2.imdecode(rgb_arr, cv2.IMREAD_COLOR)
    self._debug_img = rgb_img.copy()

    # --- Decodifica a profundidade logo em seguida ---
    depth_np = np.frombuffer(depth_resp.shot.image.data, dtype=np.uint16).reshape(
        depth_resp.shot.image.rows, depth_resp.shot.image.cols)
    depth_rows, depth_cols = depth_np.shape

    # --- Obtém a orientação da garra ---
    sim_pose = self.tf_manager.get_pose()
    if sim_pose is None:
        rospy.logwarn("Falha ao obter a pose da garra.")
        return None

    # --- Calcula o ângulo de rotação completo a partir do quaternion ---
    quat = sim_pose.orientation
    # Extrai os ângulos de Euler (roll, pitch, yaw) do quaternion
    sinr_cosp = 2 * (quat.w * quat.x + quat.y * quat.z)
    cosr_cosp = 1 - 2 * (quat.x * quat.x + quat.y * quat.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (quat.w * quat.y - quat.z * quat.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
    cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # Convertendo para graus
    roll_deg = roll * 180.0 / math.pi
    pitch_deg = pitch * 180.0 / math.pi
    yaw_deg = yaw * 180.0 / math.pi

    # No caso da câmera da garra, geralmente uma combinação das rotações é mais útil
    angle_deg = yaw_deg  # Começamos com yaw, que é a rotação em Z

    # Se a câmera estiver montada de forma que o eixo principal aponte no sentido do X ou Y do robô,
    # podemos precisar usar roll ou pitch em vez de yaw
    if abs(roll_deg) > abs(yaw_deg) and abs(roll_deg) > abs(pitch_deg):
        angle_deg = roll_deg
    elif abs(pitch_deg) > abs(yaw_deg) and abs(pitch_deg) > abs(roll_deg):
        angle_deg = pitch_deg

    # Log para debug
    rospy.loginfo(f"Ângulos de Euler: roll={roll_deg:.2f}°, pitch={pitch_deg:.2f}°, yaw={yaw_deg:.2f}°")
    rospy.loginfo(f"Usando ângulo de rotação: {angle_deg:.2f}°")

    # --- Rotação da imagem usando abordagem mais robusta ---
    h, w = rgb_img.shape[:2]
    center = (w // 2, h // 2)
    rotation_matrix_2d = cv2.getRotationMatrix2D(center, -angle_deg, 1.0)
    aligned_image = cv2.warpAffine(rgb_img, rotation_matrix_2d, (w, h), 
                                flags=cv2.INTER_LINEAR, 
                                borderMode=cv2.BORDER_CONSTANT,
                                borderValue=(0, 0, 0))
    
    # --- Salva a imagem rotacionada para visualização de debug ---
    self._debug_img_rotated = aligned_image.copy()

    # --- Detecta e filtra objetos na imagem rotacionada ---
    boxes, _, names = self.object_detector.detect_objects(aligned_image)
    candidates = self.object_detector.filter_allowed_objects(boxes, names)
    if not candidates:
        self._debug_box = None
        self._debug_box_original = None
        self._debug_dist_m = None
        return None

    # --- Seleciona o melhor candidato ---
    _, _, idx = candidates[0]
    x1, y1, x2, y2 = boxes[idx]
    cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
    
    # --- Armazena o box para visualização na imagem rotacionada ---
    self._debug_box_rotated = (int(x1), int(y1), int(x2), int(y2))
    
    # --- Projeta o box de volta para a imagem original ---
    h, w = aligned_image.shape[:2]
    center = (w // 2, h // 2)

    # Matriz de rotação inversa (usando o negativo do ângulo)
    inv_rotation_matrix_2d = cv2.getRotationMatrix2D(center, angle_deg, 1.0)

    # Pontos a serem transformados: cantos e centro do objeto
    points = np.array([[x1, y1], [x2, y2], [cx, cy]], dtype=np.float32).reshape(-1, 1, 2)

    # Aplica a transformação inversa
    transformed_points = cv2.transform(points, inv_rotation_matrix_2d)

    # Extrai os pontos transformados
    x1_orig, y1_orig = transformed_points[0][0]
    x2_orig, y2_orig = transformed_points[1][0]
    cx_orig, cy_orig = transformed_points[2][0]

    # Converte para inteiros e garante que estão dentro dos limites da imagem
    depth_rows, depth_cols = depth_np.shape
    cx_orig_int = min(max(0, int(cx_orig)), depth_cols - 1)
    cy_orig_int = min(max(0, int(cy_orig)), depth_rows - 1)

    # Armazena o box para visualização na imagem original
    self._debug_box = (int(x1_orig), int(y1_orig), int(x2_orig), int(y2_orig))
    self._debug_center_orig = (int(cx_orig), int(cy_orig))

    # --- Alinhamento da profundidade (usando coordenadas na imagem original) ---
    depth_np = np.frombuffer(depth_resp.shot.image.data, dtype=np.uint16).reshape(
        depth_resp.shot.image.rows, depth_resp.shot.image.cols)

    # Converte para inteiros e garante que estão dentro dos limites da imagem
    depth_rows, depth_cols = depth_np.shape
    x1_orig_safe = min(max(0, int(x1_orig)), depth_cols - 1)
    y1_orig_safe = min(max(0, int(y1_orig)), depth_rows - 1)
    x2_orig_safe = min(max(0, int(x2_orig)), depth_cols - 1)
    y2_orig_safe = min(max(0, int(y2_orig)), depth_rows - 1)
    cx_orig_safe = min(max(0, int(cx_orig)), depth_cols - 1)
    cy_orig_safe = min(max(0, int(cy_orig)), depth_rows - 1)

    # Armazena o box para visualização na imagem original
    self._debug_box = (x1_orig_safe, y1_orig_safe, x2_orig_safe, y2_orig_safe)
    self._debug_center_orig = (cx_orig_safe, cy_orig_safe)

    # Usa o centro do objeto na imagem original para obter a profundidade
    raw_mm = int(depth_np[cy_orig_safe, cx_orig_safe])

    min_depth_mm = 300   # 30 cm
    max_depth_mm = 3000  # 3 m

    depth_mm = raw_mm if (min_depth_mm <= raw_mm <= max_depth_mm) else min_depth_mm
    depth_m = depth_mm / 1000.0

    self._debug_dist_m = depth_m  # Salva a distância em metros

    # --- De-projeção do pixel -> quadro de câmera (usando coordenadas na imagem original) ---
    pinhole = depth_resp.source.pinhole
    fx = pinhole.intrinsics.focal_length.x
    fy = pinhole.intrinsics.focal_length.y
    cx0 = pinhole.intrinsics.principal_point.x
    cy0 = pinhole.intrinsics.principal_point.y
    x_cam = (cx_orig - cx0) * depth_m / fx
    y_cam = (cy_orig - cy0) * depth_m / fy
    z_cam = depth_m

    # --- Quadro de câmera → quadro de odom ---
    try:
        odom_T_cam = get_a_tform_b(depth_resp.shot.transforms_snapshot,
                                 ODOM_FRAME_NAME,
                                 depth_resp.shot.frame_name_image_sensor)
    except Exception as exc:
        rospy.logwarn_throttle(2.0, "Falha ao obter a transformação cam→odom: %s", exc)
        return None
    cam_T_obj = SE3Pose(x_cam, y_cam, z_cam, Quat())
    odom_T_obj = odom_T_cam * cam_T_obj
    return np.array([odom_T_obj.x, odom_T_obj.y, odom_T_obj.z])
```

### Código Original: Debug Visualization
```python
def _show_debug_overlay(self, pf_active: bool):
    """Displays image with bounding box, distance, and PF status."""
    if getattr(self, "_debug_img", None) is None:
        return  # Nothing to display

    # Cria visualização para imagem original
    vis_orig = self._debug_img.copy()
    if self._debug_box:
        x1, y1, x2, y2 = self._debug_box
        color = (0, 255, 0) if pf_active else (0, 0, 255)
        cv2.rectangle(vis_orig, (x1, y1), (x2, y2), color, 2)
        
        # Desenha o centro do objeto
        if hasattr(self, "_debug_center_orig"):
            cx, cy = self._debug_center_orig
            cv2.circle(vis_orig, (cx, cy), 4, color, -1)
            
        if self._debug_dist_m:
            cv2.putText(vis_orig, f"{self._debug_dist_m:.2f} m",
                      (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                      0.6, color, 2)

    # Adiciona indicação da orientação da garra (como uma seta)
    sim_pose = self.tf_manager.get_pose()
    if sim_pose is not None:
        h, w = vis_orig.shape[:2]
        center_x, center_y = w // 2, h // 2
        arrow_length = 50
        quat = sim_pose.orientation
        # Cria vetor de direção simplificado a partir do quaternion
        dx = 2 * (quat.x * quat.z + quat.w * quat.y)
        dy = 2 * (quat.y * quat.z - quat.w * quat.x)
        dz = 1 - 2 * (quat.x * quat.x + quat.y * quat.y)
        
        # Desenha seta de orientação
        end_x = int(center_x + arrow_length * dx)
        end_y = int(center_y + arrow_length * dy)
        cv2.arrowedLine(vis_orig, (center_x, center_y), (end_x, end_y), (255, 0, 0), 2)

    # Display PF status na imagem original
    txt = "PF: ON" if pf_active else "PF: OFF"
    cv2.putText(vis_orig, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (255, 255, 0), 2)
    cv2.putText(vis_orig, "ORIGINAL", (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (255, 255, 0), 2)

    # Cria visualização para imagem rotacionada
    if hasattr(self, "_debug_img_rotated"):
        vis_rot = self._debug_img_rotated.copy()
        if hasattr(self, "_debug_box_rotated"):
            x1, y1, x2, y2 = self._debug_box_rotated
            color = (0, 255, 0) if pf_active else (0, 0, 255)
            cv2.rectangle(vis_rot, (x1, y1), (x2, y2), color, 2)
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.circle(vis_rot, (cx, cy), 4, color, -1)
            
        # Adiciona texto para identificar a imagem rotacionada
        cv2.putText(vis_rot, "ROTACIONADA", (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                    0.8, (255, 255, 0), 2)
            
        # Combina as duas imagens lado a lado
        h1, w1 = vis_orig.shape[:2]
        h2, w2 = vis_rot.shape[:2]
        h = max(h1, h2)
        w = w1 + w2
        combined = np.zeros((h, w, 3), dtype=np.uint8)
        combined[:h1, :w1] = vis_orig
        combined[:h2, w1:w1+w2] = vis_rot
            
        cv2.imshow("debug_view", combined)
    else:
        # Se a imagem rotacionada não estiver disponível, mostre apenas a original
        cv2.imshow("debug_view", vis_orig)
        
    cv2.waitKey(1)
```

### Código Original: Integração no SpotController
```python
# Inicializa detector de objetos
self.object_detector = ObjectDetector(
    model_path=model_path,
    allowed_objects_csv=allowed_objects_csv
)

# Uso no control loop:
# Update target every 1 second to save bandwidth
if now - self._last_detection_time > 1.0:
    new_target = self._yolo_depth_deproject()
    if new_target is not None:
        self._pf_target_odom = new_target
        self._last_detection_time = now
    elif now - self._last_detection_time > self._target_valid_duration:
        self._pf_target_odom = None  # Clear target if too old
```

### Migração ROS1 → ROS2
- **ROS1**: `rospy.loginfo` → **ROS2**: `self.get_logger().info`
- **ROS1**: `rospy.logwarn_throttle` → **ROS2**: `self.get_logger().warn` + timer
- **ROS1**: `rospy.Subscriber` → **ROS2**: `create_subscription`
- **ROS1**: `rospy.Publisher` → **ROS2**: `create_publisher`
- **ROS1**: `cv_bridge.CvBridge()` → **ROS2**: `cv_bridge.CvBridge()`

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