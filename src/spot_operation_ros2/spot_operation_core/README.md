# Spot Operation Core

Pacote central do sistema Spot Operation ROS2, responsável pelo gerenciamento de clientes do robô, conexões, lease e estado geral da operação.

## 📋 Visão Geral

O `spot_operation_core` fornece funcionalidades centrais para o Spot:
- Gerenciamento de clientes do robô
- Controle de lease e conexões
- Gerenciamento de estado da operação
- Interface com Spot Driver ROS2
- Coordenação entre módulos

## 🏗️ Arquitetura

```
spot_operation_core/
├── spot_operation_core/
│   ├── __init__.py
│   ├── robot_client_manager.py
│   └── operation_state_manager.py
├── config/
│   └── core_config.yaml
├── launch/
│   └── core.launch.py
├── managers/
│   ├── client_manager.py
│   ├── lease_manager.py
│   └── state_manager.py
├── package.xml
├── setup.py
└── README.md
```

## 🚀 Funcionalidades

### **Robot Client Manager**
- Gerenciamento de conexões com o Spot
- Inicialização de clientes necessários
- Controle de lease
- Interface com Spot Driver ROS2
- Monitoramento de estado

### **Operation State Manager**
- Gerenciamento de estado da operação
- Coordenação entre módulos
- Monitoramento de performance
- Logging de eventos
- Interface de status

### **Lease Manager**
- Controle de lease do robô
- Aquisição e liberação de lease
- Monitoramento de lease
- Recuperação de falhas
- Integração com Spot Driver ROS2

## ⚙️ Configuração

### Arquivo de Configuração
```yaml
spot_operation_core:
  ros__parameters:
    # Configurações do robô
    robot:
      hostname: "192.168.80.3"
      username: "admin"
      password: "spotadmin2017"
      timeout: 20.0
      retry_attempts: 3
    
    # Configurações de lease
    lease:
      acquire_timeout: 10.0
      keepalive_interval: 1.0
      force_take: false
      return_at_exit: false
    
    # Configurações de estado
    state:
      publish_rate: 1.0
      enable_debug: false
      log_level: "info"
      save_logs: true
    
    # Configurações de clientes
    clients:
      command_client: true
      state_client: true
      image_client: true
      manipulation_client: true
      lease_client: true
```

### Parâmetros Principais

| Parâmetro | Descrição | Padrão |
|-----------|-----------|--------|
| `robot.hostname` | IP do robô | "192.168.80.3" |
| `robot.username` | Usuário do robô | "admin" |
| `lease.acquire_timeout` | Timeout de aquisição | 10.0s |
| `state.publish_rate` | Taxa de publicação | 1.0 Hz |
| `clients.command_client` | Habilitar cliente de comando | true |

## 📦 Dependências

### Dependências ROS2
- `rclpy` - Python client library
- `std_msgs` - Mensagens padrão
- `spot_operation_msgs` - Mensagens customizadas
- `tf2_ros` - Transformações

### Dependências Python
- `spot-ros2` - Spot Driver ROS2
- `numpy` - Computação numérica
- `time` - Manipulação de tempo
- `threading` - Threading

## 🔧 Instalação

### Pré-requisitos
1. ROS2 Humble instalado
2. Spot Driver ROS2 configurado
3. Rede configurada para o Spot
4. Credenciais do robô

### Build
```bash
cd ~/spot_ws
colcon build --packages-select spot_operation_core
source install/setup.bash
```

### Instalação de Dependências
```bash
# Spot Driver ROS2 já incluído no workspace
pip install numpy
```

## 🚀 Uso

### Launch Individual
```bash
ros2 launch spot_operation_core core.launch.py
```

### Como Módulo
```bash
# Incluído automaticamente nos launch files principais
ros2 launch spot_operation_launch spot_operation_full.launch.py
```

### Parâmetros Customizados
```bash
ros2 launch spot_operation_core core.launch.py \
  hostname:=192.168.1.100 \
  username:=custom_user \
  timeout:=30.0
```

## 📡 Tópicos

### Publicados
- `/operation_state` - Estado da operação
- `/robot_status` - Status do robô
- `/lease_status` - Status do lease
- `/core_debug` - Informações de debug

### Assinados
- `/robot_command` - Comandos para o robô
- `/state_request` - Solicitações de estado
- `/lease_request` - Solicitações de lease

## 🔧 Serviços

### Oferecidos
- `/get_operation_state` - Obter estado da operação
- `/acquire_lease` - Adquirir lease
- `/release_lease` - Liberar lease
- `/power_on` - Ligar robô
- `/power_off` - Desligar robô

### Clientes
- `/robot_command` - Comando para o robô
- `/robot_state` - Estado do robô
- `/robot_image` - Imagens do robô

## 🛠️ Desenvolvimento

### Estrutura de Código
```python
# Exemplo de uso do Robot Client Manager
from spot_operation_core.robot_client_manager import RobotClientManager

manager = RobotClientManager()
manager.set_hostname("192.168.80.3")
success = manager.connect()
```

### Adicionando Novos Clientes
```python
# Exemplo de cliente customizado
class CustomClient:
    def __init__(self):
        self.name = "custom_client"
    
    def initialize(self, robot):
        # Inicializar cliente
        pass
    
    def cleanup(self):
        # Limpeza
        pass
```

### Integração com Spot Driver ROS2
```python
# Exemplo de integração Spot Driver ROS2
from spot_msgs.action import RobotCommand
from rclpy.action import ActionClient

# Cliente para comandos do robô
robot_command_client = ActionClient(self, RobotCommand, '/robot_command')

# Cliente para manipulação
manipulation_client = ActionClient(self, Manipulation, '/manipulation')

# Serviços para gripper
open_gripper_client = self.create_client(Trigger, '/open_gripper')
close_gripper_client = self.create_client(Trigger, '/close_gripper')
```

## 🧪 Testes

### Testes Unitários
```bash
cd ~/spot_ws
colcon test --packages-select spot_operation_core
```

### Testes de Conexão
```bash
# Testar conexão com robô
ros2 run spot_operation_core test_connection --hostname 192.168.80.3
```

### Testes de Lease
```bash
# Testar aquisição de lease
ros2 run spot_operation_core test_lease --timeout 10.0
```

## 🐛 Troubleshooting

### Problemas Comuns

#### 1. Erro de Conexão
```
Erro: Falha ao conectar ao Spot
```
**Solução:**
- Verificar IP do robô
- Verificar conectividade de rede
- Verificar credenciais
- Verificar estado do robô

#### 2. Erro de Lease
```
Erro: Falha ao adquirir lease
```
**Solução:**
- Verificar se robô está disponível
- Verificar se lease está em uso
- Forçar aquisição se necessário
- Reiniciar robô

#### 3. Erro de Cliente
```
Erro: Cliente não inicializado
```
**Solução:**
- Verificar dependências
- Verificar configuração
- Reiniciar serviço
- Verificar logs

### Logs de Debug
```bash
# Habilitar logs detalhados
ros2 launch spot_operation_core core.launch.py debug_mode:=true

# Monitorar estado
ros2 topic echo /operation_state
ros2 topic echo /robot_status
```

## 📊 Performance

### Métricas Típicas
- **Taxa de conexão**: 99%+
- **Latência de lease**: 1-5 segundos
- **Taxa de publicação**: 1-5 Hz
- **Tempo de inicialização**: 5-15 segundos

### Otimizações
- Cache de conexões
- Pool de clientes
- Compressão de dados
- Paralelização

## 📚 Documentação Adicional

- [Spot SDK Documentation](https://dev.bostondynamics.com/docs/python/)
- [ROS2 Node Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Spot SDK Lease Management](https://dev.bostondynamics.com/docs/concepts/autonomy/lease)
- [ROS2 Services Tutorial](https://docs.ros.org/en/humble/Tutorials/Services.html)

## 🔄 Referência do Código Original

### Código Original: RobotClientManager
```python
class RobotClientManager:
    """Gerencia conexões e clientes do Spot."""
    
    def __init__(self, hostname="192.168.80.3", username="admin", password="spotadmin2017"):
        self.hostname = hostname
        self.username = username
        self.password = password
        self.sdk = None
        self.robot = None
        self.command_client = None
        self.state_client = None
        self.lease_client = None
        self.image_client = None
        self.manipulation_client = None
        self.lease_keepalive = None
        self.lease_wallet = None
        
    def connect(self):
        """Estabelece conexão com o robô e inicializa clientes necessários."""
        self.sdk = bosdyn.client.create_standard_sdk("SpotOperationSDK")
        self.robot = self.sdk.create_robot(self.hostname)
        self.robot.authenticate(self.username, self.password)
        self.robot.time_sync.wait_for_sync()
        
        if not self.robot.has_arm():
            rospy.logerr("Spot não possui braço. Abortando.")
            return False
            
        # Inicializa os clientes principais
        self.command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.state_client = self.robot.ensure_client(RobotStateClient.default_service_name)
        self.lease_client = self.robot.ensure_client(LeaseClient.default_service_name)
        self.image_client = self.robot.ensure_client(ImageClient.default_service_name)
        self.manipulation_client = self.robot.ensure_client(ManipulationApiClient.default_service_name)
        
        return True
        
    def acquire_lease(self):
        """Adquire lease para controle do robô."""
        try:
            root_lease = self.lease_client.acquire()
        except ResourceAlreadyClaimedError:
            rospy.logwarn("Lease já em uso; forçando aquisição com take().")
            root_lease = self.lease_client.take()
            
        # Configura wallet e processadores
        self.lease_wallet = LeaseWallet()
        add_lease_wallet_processors(self.command_client, self.lease_wallet)
        self.lease_wallet.add(root_lease)
        
        # Mantém lease ativo
        self.lease_keepalive = LeaseKeepAlive(self.lease_client, self.lease_wallet,
                                           must_acquire=True, return_at_exit=False)
        return self.lease_keepalive
        
    def power_on(self):
        """Liga os motores do robô."""
        return self.robot.power_on(timeout_sec=20)
        
    def get_robot_state(self):
        """Obtém o estado atual do robô."""
        return self.state_client.get_robot_state()
        
    def send_arm_command(self, odom_T_hand):
        """Envia comando de posição para o braço."""
        arm_command = RobotCommandBuilder.arm_pose_command(
            odom_T_hand.x, odom_T_hand.y, odom_T_hand.z,
            odom_T_hand.rot.w, odom_T_hand.rot.x,
            odom_T_hand.rot.y, odom_T_hand.rot.z,
            ODOM_FRAME_NAME, 0.5
        )
        return self.command_client.robot_command(arm_command)
        
    def open_gripper(self):
        """Abre a garra do robô."""
        gripper_command = RobotCommandBuilder.claw_gripper_open_command()
        return self.command_client.robot_command(gripper_command)
        
    def close_gripper(self, block=False, timeout_sec=2.0):
        """Fecha a garra. Se block=True, espera o comando terminar."""
        gripper_cmd = RobotCommandBuilder.claw_gripper_close_command()
        cmd_id = self.command_client.robot_command(gripper_cmd)
        if block:
            self._block_until_gripper_complete(cmd_id, timeout_sec)
        return cmd_id

    def _block_until_gripper_complete(self, cmd_id, timeout_sec):
        """Espera até o gripper terminar (sucesso ou fracasso)."""
        end_time = time.time() + timeout_sec
        while time.time() < end_time:
            resp = self.command_client.robot_command_feedback(cmd_id)
            fb = getattr(resp.feedback, "gripper_feedback", None)
            if fb:
                status = fb.status
                if status == fb.STATUS_GRIP_SUCCEEDED:
                    rospy.loginfo("✅ Gripper fechou com sucesso.")
                    return True
                if status == fb.STATUS_GRIP_FAILED:
                    rospy.logwarn("⚠️ Gripper falhou ao fechar.")
                    return False
            rospy.sleep(0.05)
        rospy.logwarn("⚠️ Timeout esperando gripper completar.")
        return False
```

### Código Original: Integração no SpotController
```python
# Conecta ao Spot
self.robot_manager = RobotClientManager(hostname=spot_hostname)

if not self.robot_manager.connect():
    rospy.logerr("Falha ao conectar ao Spot. Abortando.")
    return

# Uso no control loop:
def run(self):
    """Executa o loop principal de controle."""
    # Adquire lease
    with self.robot_manager.acquire_lease():
        # Liga o robô
        self.robot_manager.power_on()
        rospy.loginfo("Robô ligado. Iniciando sincronização contínua...")
        
        # Loop principal
        self._control_loop()

# Envio de comandos:
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

# Controle de gripper:
if self.current_gesture == 0:
    self.robot_manager.open_gripper()
else:
    self.robot_manager.close_gripper()

# Fechamento de gripper com bloqueio:
self.robot_manager.close_gripper(block=True, timeout_sec=2.0)
```

### Código Original: MoveItManager
```python
class MoveItManager:
    """Gerencia a interface com o MoveIt para planejamento de movimento."""
    
    def __init__(self, group_name="manipulator", end_effector_link="arm_link_fngr"):
        self.group_name = group_name
        self.end_effector_link = end_effector_link
        moveit_commander.roscpp_initialize([])
        self.group = moveit_commander.MoveGroupCommander(group_name)
        self.group.set_end_effector_link(end_effector_link)
        rospy.loginfo("MoveIt inicializado: End-effector (%s) em relação a: %s",
                     self.end_effector_link, self.group.get_pose_reference_frame())
                     
    def apply_wrist_lock(self):
        """Aplica restrições nas juntas específicas para travar o punho."""
        names = self.group.get_active_joints()
        vals = self.group.get_current_joint_values()
        
        # Trava a junta arm_wr0
        idx_wr0 = names.index("arm_wr0")
        locked_value_wr0 = vals[idx_wr0]
        rospy.loginfo("🔒 Travando arm_wr0 em %.3f rad", locked_value_wr0)
        jc_wr0 = JointConstraint(joint_name="arm_wr0",
                                 position=locked_value_wr0,
                                 tolerance_above=0.0,
                                 tolerance_below=0.0,
                                 weight=1.0)
        
        # Trava a junta arm_wr1
        idx_wr1 = names.index("arm_wr1")
        locked_value_wr1 = vals[idx_wr1]
        rospy.loginfo("🔒 Travando arm_wr1 em %.3f rad", locked_value_wr1)
        jc_wr1 = JointConstraint(joint_name="arm_wr1",
                                 position=locked_value_wr1,
                                 tolerance_above=0.0,
                                 tolerance_below=0.0,
                                 weight=1.0)
        
        # Aplica as restrições
        cs = Constraints()
        cs.joint_constraints.extend([jc_wr0, jc_wr1])
        self.group.set_path_constraints(cs)
        
    def get_current_pose(self):
        """Obtém a pose atual do end-effector."""
        rospy.sleep(0.1)  # Pequena pausa para garantir atualização
        return self.group.get_current_pose().pose
```

### Código Original: TFManager
```python
class TFManager:
    """Gerencia transformações de coordenadas usando TF."""
    
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
    def get_pose(self, target_frame="wrist", reference_frame="body", timeout=1.0):
        """Obtém a pose de um frame em relação a outro usando TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                reference_frame, target_frame, rospy.Time(0), rospy.Duration(timeout)
            )
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose.pose
        except Exception as e:
            rospy.logwarn(f"❗ TF lookup falhou: {e}")
            return None
```

### Migração ROS1 → ROS2
- **ROS1**: `rospy.loginfo` → **ROS2**: `self.get_logger().info`
- **ROS1**: `rospy.sleep` → **ROS2**: `time.sleep`
- **ROS1**: `rospy.ServiceProxy` → **ROS2**: `create_client`
- **ROS1**: `rospy.Publisher` → **ROS2**: `create_publisher`
- **ROS1**: `rospy.Subscriber` → **ROS2**: `create_subscription`
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