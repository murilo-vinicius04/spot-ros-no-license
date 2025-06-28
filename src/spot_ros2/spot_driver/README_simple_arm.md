# Driver Simplificado do Braço do Spot 🦾

Este driver permite controlar apenas o braço do Spot **sem precisar da licença de baixo nível**! 

## 🎯 Objetivo

O driver principal (`spot_hardware_interface`) precisa da licença de controle de baixo nível da Boston Dynamics para funcionar. Esta solução contorna essa limitação, permitindo controlar o braço usando apenas a SDK padrão.

## ✅ O que funciona

- ✅ Controle de posição das juntas do braço
- ✅ Controle de pose cartesiana da mão
- ✅ Comandos básicos: stow, unstow, carry
- ✅ Publicação do estado das juntas
- ✅ Funciona com SDK padrão (sem licença de baixo nível)

## ❌ Limitações

- ❌ Não suporta controle de corpo/pernas
- ❌ Não suporta streaming de comandos em alta frequência
- ❌ Movimentos são ponto-a-ponto (não trajetórias contínuas)
- ❌ Sem controle de ganhos das juntas

## 🚀 Como usar

### 1. Configurar variáveis de ambiente

```bash
export BOSDYN_CLIENT_USERNAME=seu_usuario
export BOSDYN_CLIENT_PASSWORD=sua_senha  
export SPOT_IP=192.168.80.3  # IP do seu Spot
```

### 2. Iniciar o driver simplificado

```bash
# Usar variáveis de ambiente
ros2 launch spot_driver simple_arm_driver.launch.py

# Ou especificar parâmetros diretamente
ros2 launch spot_driver simple_arm_driver.launch.py \
    username:=seu_usuario \
    password:=sua_senha \
    hostname:=192.168.80.3 \
    spot_name:=MeuSpot
```

### 3. Testar com o exemplo

Em outro terminal:

```bash
ros2 run spot_examples simple_arm_example.py
```

## 📡 Tópicos e Serviços

### Tópicos publicados:
- `/{spot_name}/arm_joint_states` - Estado atual das juntas do braço

### Tópicos subscritos:
- `/{spot_name}/arm_joint_commands` - Comandos de posição das juntas
- `/{spot_name}/arm_pose_commands` - Comandos de pose cartesiana

### Serviços:
- `/{spot_name}/arm_stow` - Guardar o braço
- `/{spot_name}/arm_unstow` - Preparar o braço 
- `/{spot_name}/arm_carry` - Modo carry

## 🎮 Exemplos de comandos

### Comando de juntas via linha de comando:

```bash
ros2 topic pub /{spot_name}/arm_joint_commands sensor_msgs/JointState "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
name: ['Spot/arm_sh0', 'Spot/arm_sh1', 'Spot/arm_el0', 'Spot/arm_el1', 'Spot/arm_wr0', 'Spot/arm_wr1']
position: [0.5, -1.0, 2.0, 0.0, -0.5, 0.0]
velocity: []
effort: []"
```

### Comando de pose via linha de comando:

```bash
ros2 topic pub /{spot_name}/arm_pose_commands geometry_msgs/PoseStamped "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: 'body'
pose:
  position: {x: 0.6, y: 0.2, z: 0.3}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

### Chamar serviços:

```bash
# Preparar braço
ros2 service call /{spot_name}/arm_unstow std_srvs/Trigger

# Guardar braço  
ros2 service call /{spot_name}/arm_stow std_srvs/Trigger

# Modo carry
ros2 service call /{spot_name}/arm_carry std_srvs/Trigger
```

## 🔧 Programação em Python

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        self.pub = self.create_publisher(
            JointState, 
            '/Spot/arm_joint_commands', 
            10
        )
        
    def move_arm(self, positions):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'Spot/arm_sh0', 'Spot/arm_sh1', 'Spot/arm_el0',
            'Spot/arm_el1', 'Spot/arm_wr0', 'Spot/arm_wr1'
        ]
        msg.position = positions
        self.pub.publish(msg)

# Usar
rclpy.init()
controller = ArmController()
controller.move_arm([0.0, -0.9, 1.8, 0.0, -0.9, 0.0])  # Posição unstow
rclpy.spin(controller)
```

## 📏 Limites das Juntas

| Junta | Nome | Faixa (rad) | Descrição |
|-------|------|-------------|-----------|
| sh0 | Ombro rotação | -3.14 a 3.14 | Rotação base do braço |
| sh1 | Ombro elevação | -3.13 a 0.4 | Elevação do braço |
| el0 | Cotovelo | 0.0 a 3.14 | Flexão do cotovelo |
| el1 | Antebraço | -2.79 a 2.79 | Rotação do antebraço |
| wr0 | Pulso flexão | -1.83 a 1.83 | Flexão do pulso |
| wr1 | Pulso rotação | -2.87 a 2.87 | Rotação do pulso |

## 🛠️ Solução de Problemas

### "Spot não conectado"
- Verifique o IP do robô
- Confirme usuário e senha
- Teste conexão: `ping 192.168.80.3`

### "Este Spot não possui um braço"
- Verifique se o Spot tem braço instalado
- Confirme que está conectando ao Spot correto

### "Spot arm não disponível"
- Aguarde alguns segundos após iniciar o driver
- Verifique se não há erros de autenticação

### Movimentos não executam
- Certifique-se que o Spot está ligado
- Verifique se não está em estop
- Use o serviço `arm_unstow` primeiro

## 🎉 Vantagens desta solução

1. **Sem licença de baixo nível**: Funciona com qualquer Spot
2. **Simples**: Usa apenas a SDK padrão
3. **Compatível**: Interface ROS familiar  
4. **Extensível**: Fácil de modificar e expandir
5. **Rápido**: Deploy sem complicações

## 🔄 Comparação com driver completo

| Recurso | Driver Completo | Driver Simplificado |
|---------|----------------|-------------------|
| Licença necessária | ✅ Baixo nível | ❌ Apenas padrão |
| Controle do corpo | ✅ | ❌ |
| Controle do braço | ✅ | ✅ |
| Streaming contínuo | ✅ | ❌ |
| Ganhos de junta | ✅ | ❌ |
| Facilidade de uso | ⚠️ Complexo | ✅ Simples |

## 🤝 Contribuições

Sinta-se livre para melhorar este driver! Algumas ideias:

- [ ] Suporte a trajetórias multi-ponto
- [ ] Interface para controle do gripper  
- [ ] Validação de limites de segurança
- [ ] Integração com MoveIt
- [ ] Controle de velocidade das juntas

---

**Importante**: Este driver é uma solução alternativa para quando não se tem a licença de baixo nível. Para controle completo do robô, recomenda-se obter a licença oficial da Boston Dynamics. 