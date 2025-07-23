# Spot Operation Control

Pacote de controle do braço e movimento do sistema Spot Operation ROS2, responsável pelo controle do braço, campos de potencial, detecção de colisão e integração com MoveIt2.

## 📋 Visão Geral

O `spot_operation_control` fornece capacidades de controle para o Spot:
- Controle do braço com MoveIt2
- Campos de potencial para navegação
- Detecção de colisão
- Múltiplos modos de operação
- Controle de gripper
- Integração com gestos e grasp

## 🏗️ Arquitetura

```
spot_operation_control/
├── spot_operation_control/
│   ├── __init__.py
│   └── spot_controller.py
├── config/
│   └── control_config.yaml
├── launch/
│   └── control.launch.py
├── controllers/
│   ├── arm_controller.py
│   ├── potential_field.py
│   └── collision_detector.py
├── package.xml
├── setup.py
└── README.md
```

## 🚀 Funcionalidades

### **Spot Controller**
- Controle principal do braço
- Integração com MoveIt2
- Gerenciamento de modos de operação
- Coordenação entre subsistemas
- Interface com gestos e grasp

### **Arm Controller**
- Controle de posição do braço
- Planejamento de trajetórias
- Execução de movimentos
- Controle de velocidade
- Integração com sensores

### **Potential Field**
- Campos de potencial atrativos
- Campos de potencial repulsivos
- Navegação baseada em campos
- Evitação de obstáculos
- Otimização de trajetória

### **Collision Detector**
- Detecção de colisão em tempo real
- Monitoramento de proximidade
- Parada de emergência
- Análise de workspace
- Integração com sensores

## ⚙️ Configuração

### Arquivo de Configuração
```yaml
spot_operation_control:
  ros__parameters:
    # Configurações do MoveIt
    moveit:
      group_name: "manipulator"
      end_effector_link: "arm_link_fngr"
      planning_time: 5.0
      max_velocity: 0.5
      max_acceleration: 0.3
    
    # Configurações de controle
    control:
      control_rate: 5.0
      position_tolerance: 0.01
      orientation_tolerance: 0.1
      enable_gravity_compensation: true
    
    # Configurações de campo de potencial
    potential_field:
      k_att: 0.2
      k_rep: 0.5
      attraction_distance: 0.4
      repulsion_distance: 0.2
      max_force: 10.0
    
    # Configurações de colisão
    collision:
      safety_distance: 0.1
      enable_collision_detection: true
      emergency_stop_distance: 0.05
      publish_collision_markers: true
    
    # Configurações de gripper
    gripper:
      max_force: 50.0
      max_velocity: 0.1
      position_tolerance: 0.005
```

### Parâmetros Principais

| Parâmetro | Descrição | Padrão |
|-----------|-----------|--------|
| `moveit.group_name` | Nome do grupo MoveIt | "manipulator" |
| `moveit.end_effector_link` | Link do end-effector | "arm_link_fngr" |
| `control.control_rate` | Taxa de controle | 5.0 Hz |
| `potential_field.k_att` | Ganho de atração | 0.2 |
| `collision.safety_distance` | Distância de segurança | 0.1m |

## 📦 Dependências

### Dependências ROS2
- `rclpy` - Python client library
- `geometry_msgs` - Mensagens de geometria
- `trajectory_msgs` - Mensagens de trajetória
- `sensor_msgs` - Mensagens de sensores
- `moveit_msgs` - Mensagens do MoveIt
- `spot_operation_msgs` - Mensagens customizadas

### Dependências Python
- `moveit_py` - MoveIt Python API
- `numpy` - Computação numérica
- `scipy` - Computação científica
- `tf2_ros` - Transformações
- `spot-ros2` - Spot Driver ROS2

## 🔧 Instalação

### Pré-requisitos
1. ROS2 Humble instalado
2. MoveIt2 configurado
3. Spot Driver ROS2 configurado
4. URDF do Spot configurado

### Build
```bash
cd ~/spot_ws
colcon build --packages-select spot_operation_control
source install/setup.bash
```

## 🚀 Uso

### Launch Individual
```bash
ros2 launch spot_operation_control control.launch.py
```

### Como Módulo
```bash
# Incluído automaticamente nos launch files principais
ros2 launch spot_operation_launch spot_operation_full.launch.py
```

### Parâmetros Customizados
```bash
ros2 launch spot_operation_control control.launch.py \
  group_name:=manipulator \
  control_rate:=10.0 \
  k_att:=0.3 \
  safety_distance:=0.15
```

## 📡 Tópicos

### Publicados
- `/arm_state` - Estado do braço
- `/collision_warning` - Avisos de colisão
- `/potential_field` - Campo de potencial
- `/gripper_state` - Estado do gripper
- `/control_debug` - Informações de debug

### Assinados
- `/gesture_command` - Comandos de gesto
- `/grasp_results` - Resultados de grasp
- `/move_group/status` - Status do MoveIt
- `/joint_states` - Estados das juntas

## 🔧 Serviços

### Oferecidos
- `/set_control_mode` - Alterar modo de controle
- `/get_arm_state` - Obter estado do braço
- `/set_gripper_position` - Controlar gripper
- `/emergency_stop` - Parada de emergência

### Clientes
- `/move_group/plan` - Planejar movimento
- `/move_group/execute` - Executar movimento
- `/grasp_execute` - Executar grasp

## 🎯 Actions

### Oferecidos
- `/move_arm` - Mover braço
- `/follow_trajectory` - Seguir trajetória
- `/grasp_object` - Grasp de objeto

## 🛠️ Desenvolvimento

### Estrutura de Código
```python
# Exemplo de uso do Spot Controller
from spot_operation_control.spot_controller import SpotController

controller = SpotController()
controller.set_mode("autonomous")
success = controller.move_to_pose(target_pose)
```

### Adicionando Novos Modos
```python
# Exemplo de modo customizado
class CustomMode:
    def __init__(self):
        self.name = "custom_mode"
    
    def execute(self, command):
        # Implementar lógica do modo
        return success
    
    def cleanup(self):
        # Limpeza ao sair do modo
        pass
```

### Integração com MoveIt
```python
# Exemplo de integração MoveIt
from moveit_py import MoveItPy

moveit = MoveItPy("manipulator")
target_pose = geometry_msgs.msg.PoseStamped()
success = moveit.plan_and_execute(target_pose)
```

## 🧪 Testes

### Testes Unitários
```bash
cd ~/spot_ws
colcon test --packages-select spot_operation_control
```

### Testes de Movimento
```bash
# Testar movimento simples
ros2 run spot_operation_control test_movement --pose "0.5, 0.0, 0.3"
```

### Testes de Colisão
```bash
# Testar detecção de colisão
ros2 run spot_operation_control test_collision --obstacle_position "0.3, 0.0, 0.2"
```

## 🐛 Troubleshooting

### Problemas Comuns

#### 1. Erro de MoveIt
```
Erro: Falha no planejamento MoveIt
```
**Solução:**
- Verificar configuração do URDF
- Verificar workspace do braço
- Verificar colisões
- Ajustar parâmetros de planejamento

#### 2. Erro de Colisão
```
Erro: Detecção de colisão ativa
```
**Solução:**
- Verificar sensores
- Ajustar distância de segurança
- Verificar ambiente
- Limpar obstáculos

#### 3. Erro de Controle
```
Erro: Falha no controle do braço
```
**Solução:**
- Verificar estado do robô
- Verificar lease
- Verificar sensores
- Reiniciar controle

### Logs de Debug
```bash
# Habilitar logs detalhados
ros2 launch spot_operation_control control.launch.py debug_mode:=true

# Monitorar estado
ros2 topic echo /arm_state
ros2 topic echo /collision_warning
```

## 📊 Performance

### Métricas Típicas
- **Taxa de controle**: 5-10 Hz
- **Latência de movimento**: 100-500ms
- **Precisão de posição**: ±1cm
- **Tempo de planejamento**: 1-3 segundos

### Otimizações
- Cache de planejamentos
- Redução de resolução
- Paralelização de cálculos
- Uso de GPU para campos de potencial

## 📚 Documentação Adicional

- [MoveIt2 Documentation](https://moveit.ros.org/)
- [ROS2 Control](https://control.ros.org/)
- [Potential Field Navigation](https://en.wikipedia.org/wiki/Potential_field_method)
- [Spot SDK Arm Control](https://dev.bostondynamics.com/docs/concepts/arm/arm_control)

## 🔄 Referência do Código Original

### Código Original: SpotController
```python
class SpotController:
    """Controller with light potential field support in MANUAL mode."""
    
    def __init__(self, spot_hostname="192.168.80.3",
                 model_path="/root/ws_moveit/src/spot_operation/scripts/yolo11n.pt",
                 allowed_objects_csv="/root/ws_moveit/src/spot_operation/config/allowed_objects.csv"):
        # Inicializa ROS
        rospy.init_node("continuous_moveit_pose_to_spot_real", anonymous=True)
        
        # Configuração de gestos
        self.current_gesture = 0
        self.manipulation_mode = False
        rospy.Subscriber("/hand_gesture", Int32, self._gesture_callback)
        
        # Inicializa gerenciadores
        self.tf_manager = TFManager()
        self.moveit_manager = MoveItManager()
        
        # Aplica restrições de juntas
        self.moveit_manager.apply_wrist_lock()
        
        # Conecta ao Spot
        self.robot_manager = RobotClientManager(hostname=spot_hostname)
        
        if not self.robot_manager.connect():
            rospy.logerr("Falha ao conectar ao Spot. Abortando.")
            return
            
        # Inicializa detector de objetos
        self.object_detector = ObjectDetector(
            model_path=model_path,
            allowed_objects_csv=allowed_objects_csv
        )
        
        # Inicializa gerenciador de grasp
        self.grasp_manager = GraspManager()
        
        # Registra múltiplas estratégias de grasp
        front_grasp = YOLOGraspStrategy(
            detector=self.object_detector,
            image_source="hand_color_image"
        )
        self.grasp_manager.register_strategy("front_yolo", front_grasp)
        
        side_grasp = YOLOGraspStrategy(
            detector=self.object_detector,
            image_source="frontleft_fisheye_image"
        )
        self.grasp_manager.register_strategy("side_yolo", side_grasp)
        
        self.default_grasp_strategy = "front_yolo"
        self.fallback_grasp_strategy = "side_yolo"

        # Select operation mode using the service
        finger_client = FingerCountClient()
        mode_code = finger_client.request_mode()  # 1 or 2
        self.mode = "manual" if mode_code == 1 else "semi"
        rospy.loginfo("🚀 Operation mode selected: %s", self.mode.upper())

        # PF state
        self._pf_target_odom = None
        self._last_detection_time = 0.0
        self._target_valid_duration = 3.0
        self._is_colliding = False
        rospy.Subscriber("/move_group/monitored_planning_scene",
                         PlanningScene, self._ps_callback,
                          queue_size=1)
```

### Código Original: Control Loop
```python
def _control_loop(self):
    """Main control loop."""
    AUTONOMOUS = "autonomous"
    MANUAL     = "manual"
    self._inner_state = MANUAL

    # Ensure safety variables are initialized
    self.frozen_orientation = None
    self.is_orientation_locked = False

    rate = rospy.Rate(5)  # 5 Hz gives smoother PF updates

    # No começo do _control_loop()
    finger_client = FingerCountClient()

    while not rospy.is_shutdown():
        now = time.time()
        self.mode = finger_client.check_and_update_mode(self.mode)
        
        # -------------- MANUAL (MODE 1) --------------------------------
        if self.mode == "manual":
            # Update target every 1 second to save bandwidth
            if now - self._last_detection_time > 1.0:
                new_target = self._yolo_depth_deproject()
                if new_target is not None:
                    self._pf_target_odom = new_target
                    self._last_detection_time = now
                elif now - self._last_detection_time > self._target_valid_duration:
                    self._pf_target_odom = None

            # Gripper command based on gesture
            if self.current_gesture == 0:
                self.robot_manager.open_gripper()
            else:
                self.robot_manager.close_gripper()

            sim_pose = self.tf_manager.get_pose()
            if sim_pose is None:
                rate.sleep()
                continue

            # Build current pose
            current_pos = np.array([sim_pose.position.x,
                                     sim_pose.position.y,
                                     sim_pose.position.z])
            target_pos = self._pf_target_odom

            use_pf = (target_pos is not None and self.current_gesture == 0)

            if use_pf:
                dist = np.linalg.norm(target_pos - current_pos)
                # atrai só se estiver MAIS PERTO que 40 cm
                if dist < 0.40:
                    delta = compute_attractive(current_pos, target_pos, k_att=0.2)
                    new_pos = current_pos + delta
                else:
                    new_pos = current_pos
            else:
                new_pos = current_pos

            # Send command
            robot_state = self.robot_manager.get_robot_state()
            odom_T_body = get_a_tform_b(robot_state.kinematic_state.transforms_snapshot,
                                        ODOM_FRAME_NAME, "body")
            quat_to_use = (self.frozen_orientation if self.is_orientation_locked
                           else sim_pose.orientation)
            flat_body_T_hand = SE3Pose(x=new_pos[0], y=new_pos[1], z=new_pos[2],
                                       rot=Quat(w=quat_to_use.w,
                                                x=quat_to_use.x,
                                                y=quat_to_use.y,
                                                z=quat_to_use.z))
            odom_T_hand = odom_T_body * flat_body_T_hand
            self.robot_manager.send_arm_command(odom_T_hand)

            self._show_debug_overlay(use_pf)
            rate.sleep()
            continue
```

### Código Original: Grasp Execution
```python
# Verifica gestos para iniciar grasp
if self.current_gesture == 1:
    rospy.loginfo("✋ Gesto de agarrar detectado!")
    
    # Congela orientação atual antes de fechar a garra
    sim_pose = self.tf_manager.get_pose()
    if sim_pose:
        self.frozen_orientation = sim_pose.orientation
        self.is_orientation_locked = True
        rospy.loginfo("🔒 Orientação congelada durante fechamento da garra")

    # Fecha a garra e espera resposta
    self.robot_manager.close_gripper(block=True, timeout_sec=2.0)

    rospy.loginfo("✅ Garra fechada. Voltando a liberar orientação")
    self.is_orientation_locked = False

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

### Código Original: Potential Field
```python
def compute_attractive(current: np.ndarray, target: np.ndarray, k_att: float = 0.2) -> np.ndarray:
    """Compute a gentle attractive force.

    Args:
        current: np.array([x, y, z]) – end‑effector in odom.
        target : np.array([x, y, z]) – object position in odom.
        k_att  : small gain.
    Returns:
        np.array – delta vector toward the target.
    """
    return k_att * (target - current)
```

### Código Original: Collision Detection
```python
def _ps_callback(self, msg):
    # O campo world.collision_objects fica vazio quando NÃO há colisão
    self._is_colliding = len(msg.world.collision_objects) > 0

def _escape_via_moveit(self):
    """Planeja 20 cm para cima (eixo –Z do EE) e executa."""
    current = self.moveit_manager.get_current_pose()
    target  = copy.deepcopy(current)
    target.position.z += 0.20          # 20 cm

    group = self.moveit_manager.group
    group.set_pose_target(target)

    success, plan, _ = group.plan()
    if success and plan.joint_trajectory.points:
        rospy.loginfo("Executando trajeto de escape...")
        group.execute(plan, wait=True)
        group.stop()
        group.clear_pose_targets()
    else:
        rospy.logerr("MoveIt não conseguiu uma trajetória livre!")
```

### Migração ROS1 → ROS2
- **ROS1**: `rospy.init_node` → **ROS2**: `rclpy.init` + `Node.__init__`
- **ROS1**: `rospy.Subscriber` → **ROS2**: `create_subscription`
- **ROS1**: `rospy.Publisher` → **ROS2**: `create_publisher`
- **ROS1**: `rospy.Rate` → **ROS2**: `create_timer`
- **ROS1**: `rospy.loginfo` → **ROS2**: `self.get_logger().info`
- **ROS1**: `moveit_commander` → **ROS2**: `moveit_py`

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