#!/usr/bin/env python3

# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

"""
Driver simplificado para controlar apenas o braço do Spot sem licença de baixo nível.

Este driver usa a SDK padrão através do spot_wrapper para controlar o braço,
permitindo movimento das juntas sem precisar da licença de controle de baixo nível.
"""

import logging
import time
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped

from spot_wrapper.wrapper import SpotWrapper


class SimpleArmDriver(Node):
    """Driver simplificado para controle do braço do Spot."""
    
    def __init__(self):
        super().__init__('simple_arm_driver')
        
        self.get_logger().info("🤖 Inicializando driver simplificado do braço do Spot...")
        
        # Declarar parâmetros
        self.declare_parameter('username', 'user')
        self.declare_parameter('password', 'password') 
        self.declare_parameter('hostname', '10.0.0.3')
        self.declare_parameter('port', 0)
        self.declare_parameter('spot_name', 'Spot')
        self.declare_parameter('state_rate', 10.0)  # Hz para publicar estado das juntas
        
        # Obter parâmetros
        self.username = self.get_parameter('username').value
        self.password = self.get_parameter('password').value
        self.hostname = self.get_parameter('hostname').value
        self.port = self.get_parameter('port').value
        self.spot_name = self.get_parameter('spot_name').value
        self.state_rate = self.get_parameter('state_rate').value
        
        # Logger para o wrapper
        self.wrapper_logger = logging.getLogger(f"{self.spot_name}.spot_wrapper")
        
        # Callback group para operações concorrentes
        self.callback_group = MutuallyExclusiveCallbackGroup()
        
        # Inicializar o SpotWrapper
        self.spot_wrapper: Optional[SpotWrapper] = None
        self._initialize_spot_wrapper()
        
        # Estado das juntas do braço
        self.arm_joint_names = ['arm_sh0', 'arm_sh1', 'arm_el0', 'arm_el1', 'arm_wr0', 'arm_wr1']
        self.current_joint_positions = [0.0] * 6
        self.current_joint_velocities = [0.0] * 6
        self.current_joint_efforts = [0.0] * 6
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, 
            f'/{self.spot_name}/arm_joint_states', 
            10
        )
        
        # Subscribers
        self.joint_command_sub = self.create_subscription(
            JointState,
            f'/{self.spot_name}/arm_joint_commands',
            self.joint_command_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.pose_command_sub = self.create_subscription(
            PoseStamped,
            f'/{self.spot_name}/arm_pose_commands', 
            self.pose_command_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Services
        self.arm_stow_srv = self.create_service(
            Trigger, 
            f'/{self.spot_name}/arm_stow',
            self.arm_stow_callback,
            callback_group=self.callback_group
        )
        
        self.arm_unstow_srv = self.create_service(
            Trigger,
            f'/{self.spot_name}/arm_unstow', 
            self.arm_unstow_callback,
            callback_group=self.callback_group
        )
        
        self.arm_carry_srv = self.create_service(
            Trigger,
            f'/{self.spot_name}/arm_carry',
            self.arm_carry_callback, 
            callback_group=self.callback_group
        )
        
        # Timer para publicar estado das juntas
        self.state_timer = self.create_timer(
            1.0 / self.state_rate,
            self.publish_joint_states
        )
        
        self.get_logger().info("✅ Driver simplificado do braço iniciado com sucesso!")
        self.get_logger().info(f"🎯 Tópicos disponíveis:")
        self.get_logger().info(f"   📤 Estado: /{self.spot_name}/arm_joint_states")
        self.get_logger().info(f"   📥 Comandos: /{self.spot_name}/arm_joint_commands")
        self.get_logger().info(f"   📥 Pose: /{self.spot_name}/arm_pose_commands") 
        self.get_logger().info(f"🛠️  Serviços disponíveis:")
        self.get_logger().info(f"   📋 /{self.spot_name}/arm_stow")
        self.get_logger().info(f"   📋 /{self.spot_name}/arm_unstow") 
        self.get_logger().info(f"   📋 /{self.spot_name}/arm_carry")
        
    def _initialize_spot_wrapper(self):
        """Inicializa o SpotWrapper."""
        try:
            self.get_logger().info(f"🔗 Conectando ao Spot em {self.hostname}...")
            
            self.spot_wrapper = SpotWrapper(
                username=self.username,
                password=self.password,
                hostname=self.hostname,
                port=self.port,
                robot_name=self.spot_name,
                logger=self.wrapper_logger,
                start_estop=False,  # Não iniciar estop automaticamente
                rates={'robot_state': self.state_rate},  # Para obter estado das juntas
                callbacks={'robot_state': self._robot_state_callback}
            )
            
            if not self.spot_wrapper.is_valid:
                raise RuntimeError("Falha ao conectar com o Spot")
                
            if not self.spot_wrapper.has_arm():
                raise RuntimeError("Este Spot não possui um braço")
                
            self.get_logger().info("✅ Conexão com Spot estabelecida!")
            
        except Exception as e:
            self.get_logger().error(f"❌ Erro ao conectar com Spot: {e}")
            self.spot_wrapper = None
            
    def _robot_state_callback(self, future):
        """Callback para atualizar estado do robô."""
        if not self.spot_wrapper:
            return
            
        try:
            robot_state = future.result()
            if robot_state and robot_state.kinematic_state:
                # Extrair posições das juntas do braço
                joint_states = robot_state.kinematic_state.joint_states
                
                for i, joint_name in enumerate(['sh0', 'sh1', 'el0', 'el1', 'wr0', 'wr1']):
                    # Buscar joint pelo nome (pode ter prefixo)
                    for joint in joint_states:
                        if joint_name in joint.name:
                            if i < len(self.current_joint_positions):
                                self.current_joint_positions[i] = joint.position.value
                                self.current_joint_velocities[i] = joint.velocity.value  
                                self.current_joint_efforts[i] = joint.load.value
                            break
        except Exception as e:
            self.get_logger().debug(f"Erro ao processar estado do robô: {e}")
            
    def publish_joint_states(self):
        """Publica o estado atual das juntas do braço."""
        if not self.spot_wrapper:
            return
            
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"{self.spot_name}/body"
        
        msg.name = [f"{self.spot_name}/{name}" for name in self.arm_joint_names]
        msg.position = self.current_joint_positions.copy()
        msg.velocity = self.current_joint_velocities.copy() 
        msg.effort = self.current_joint_efforts.copy()
        
        self.joint_state_pub.publish(msg)
        
    def joint_command_callback(self, msg: JointState):
        """Callback para comandos de juntas."""
        if not self.spot_wrapper:
            self.get_logger().warning("⚠️  Spot não conectado - ignorando comando")
            return
            
        self.get_logger().info(f"📥 Recebido comando de juntas: {len(msg.position)} posições")
        
        try:
            # Mapear nomes das juntas para posições
            joint_positions = {}
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    # Extrair nome da junta (remover prefixos)
                    clean_name = name.split('/')[-1]  # Pega apenas a parte após '/'
                    if 'sh0' in clean_name:
                        joint_positions['sh0'] = msg.position[i]
                    elif 'sh1' in clean_name:
                        joint_positions['sh1'] = msg.position[i]
                    elif 'el0' in clean_name:
                        joint_positions['el0'] = msg.position[i]
                    elif 'el1' in clean_name:
                        joint_positions['el1'] = msg.position[i]
                    elif 'wr0' in clean_name:
                        joint_positions['wr0'] = msg.position[i]
                    elif 'wr1' in clean_name:
                        joint_positions['wr1'] = msg.position[i]
            
            # Verificar se temos todas as 6 juntas
            required_joints = ['sh0', 'sh1', 'el0', 'el1', 'wr0', 'wr1']
            if not all(joint in joint_positions for joint in required_joints):
                self.get_logger().warning(f"⚠️  Comando incompleto. Precisa de 6 juntas: {required_joints}")
                return
                
            # Enviar comando para o braço
            success, message = self.spot_wrapper.arm_joint_cmd(**joint_positions)
            
            if success:
                self.get_logger().info("✅ Comando de juntas enviado com sucesso")
            else:
                self.get_logger().error(f"❌ Erro ao enviar comando: {message}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Erro no callback de comando de juntas: {e}")
            
    def pose_command_callback(self, msg: PoseStamped):
        """Callback para comandos de pose."""
        if not self.spot_wrapper or not self.spot_wrapper.spot_arm:
            self.get_logger().warning("⚠️  Spot arm não disponível - ignorando comando de pose")
            return
            
        self.get_logger().info(f"📥 Recebido comando de pose para frame: {msg.header.frame_id}")
        
        try:
            success, message = self.spot_wrapper.spot_arm.hand_pose(
                x=msg.pose.position.x,
                y=msg.pose.position.y,
                z=msg.pose.position.z,
                qx=msg.pose.orientation.x,
                qy=msg.pose.orientation.y,
                qz=msg.pose.orientation.z,
                qw=msg.pose.orientation.w,
                ref_frame=msg.header.frame_id or "body",
                ensure_power_on_and_stand=True,
                blocking=False
            )
            
            if success:
                self.get_logger().info("✅ Comando de pose enviado com sucesso")
            else:
                self.get_logger().error(f"❌ Erro ao enviar comando de pose: {message}")
                
        except Exception as e:
            self.get_logger().error(f"❌ Erro no callback de pose: {e}")
            
    def arm_stow_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Service callback para guardar o braço."""
        if not self.spot_wrapper or not self.spot_wrapper.spot_arm:
            response.success = False
            response.message = "Spot arm não disponível"
            return response
            
        try:
            success, message = self.spot_wrapper.spot_arm.arm_stow()
            response.success = success
            response.message = message
            
            if success:
                self.get_logger().info("✅ Braço guardado com sucesso")
            else:
                self.get_logger().error(f"❌ Erro ao guardar braço: {message}")
                
        except Exception as e:
            response.success = False
            response.message = f"Erro: {e}"
            self.get_logger().error(f"❌ Erro no serviço arm_stow: {e}")
            
        return response
        
    def arm_unstow_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Service callback para preparar o braço."""
        if not self.spot_wrapper or not self.spot_wrapper.spot_arm:
            response.success = False
            response.message = "Spot arm não disponível"
            return response
            
        try:
            success, message = self.spot_wrapper.spot_arm.arm_unstow()
            response.success = success
            response.message = message
            
            if success:
                self.get_logger().info("✅ Braço preparado com sucesso")
            else:
                self.get_logger().error(f"❌ Erro ao preparar braço: {message}")
                
        except Exception as e:
            response.success = False
            response.message = f"Erro: {e}"
            self.get_logger().error(f"❌ Erro no serviço arm_unstow: {e}")
            
        return response
        
    def arm_carry_callback(self, request: Trigger.Request, response: Trigger.Response):
        """Service callback para modo carry do braço.""" 
        if not self.spot_wrapper or not self.spot_wrapper.spot_arm:
            response.success = False
            response.message = "Spot arm não disponível"
            return response
            
        try:
            success, message = self.spot_wrapper.spot_arm.arm_carry()
            response.success = success
            response.message = message
            
            if success:
                self.get_logger().info("✅ Modo carry ativado com sucesso")
            else:
                self.get_logger().error(f"❌ Erro ao ativar modo carry: {message}")
                
        except Exception as e:
            response.success = False
            response.message = f"Erro: {e}"
            self.get_logger().error(f"❌ Erro no serviço arm_carry: {e}")
            
        return response


def main(args=None):
    """Função principal do driver."""
    rclpy.init(args=args)
    
    try:
        driver = SimpleArmDriver()
        rclpy.spin(driver)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Erro no driver: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main() 