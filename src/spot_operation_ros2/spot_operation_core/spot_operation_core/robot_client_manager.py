#!/usr/bin/env python3
"""
Gerenciador de estado da operação do Spot.
Usando Spot Driver ROS2.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time
import threading
from typing import Optional, Dict, Any

from spot_operation_msgs.msg import OperationState
from spot_operation_msgs.srv import GetOperationState

# Spot Driver ROS2 imports
from spot_msgs.msg import RobotState, ManipulationState
from std_srvs.srv import Trigger

class OperationStateManager(Node):
    """Gerencia o estado da operação usando Spot Driver ROS2."""
    
    def __init__(self):
        super().__init__('operation_state_manager')
        
        # Estado da operação
        self.is_connected = False
        self.is_powered_on = False
        self.has_lease = False
        self.current_mode = "unknown"
        self.current_strategy = "none"
        self.success_rate = 0.0
        
        # Publishers e Services
        self.state_publisher = self.create_publisher(
            OperationState, 'operation_state', 10
        )
        self.state_service = self.create_service(
            GetOperationState, 'get_operation_state', self.get_operation_state_callback
        )
        
        # SPOT DRIVER ROS2 - Subscribers
        self.robot_state_sub = self.create_subscription(
            RobotState, '/robot_state', self.robot_state_callback, 10
        )
        self.manipulation_state_sub = self.create_subscription(
            ManipulationState, '/manipulation_state', self.manipulation_state_callback, 10
        )
        
        # Timer para publicação do estado
        self.state_timer = self.create_timer(1.0, self.publish_state)
        
        self.get_logger().info("OperationStateManager inicializado")
        
    def robot_state_callback(self, msg: RobotState):
        """Callback para estado do robô via Spot Driver."""
        # Atualiza estado baseado nas mensagens do Spot Driver
        self.is_connected = True  # Se recebe mensagem, está conectado
        self.is_powered_on = msg.power_state.motor_power_state == 1  # POWER_STATE_ON
        self.has_lease = msg.lease_owner.client_name != ""  # Se tem lease owner
        
    def manipulation_state_callback(self, msg: ManipulationState):
        """Callback para estado de manipulação via Spot Driver."""
        # Atualiza estado de manipulação
        if msg.manipulation_state == 1:  # MANIP_STATE_GRASP_SUCCEEDED
            self.current_strategy = "grasp_success"
        elif msg.manipulation_state == 2:  # MANIP_STATE_GRASP_FAILED
            self.current_strategy = "grasp_failed"
        else:
            self.current_strategy = "idle"
        
    def get_operation_state_callback(self, request, response):
        """Callback para o serviço GetOperationState."""
        response.state.header.stamp = self.get_clock().now().to_msg()
        response.state.current_mode = self.current_mode
        response.state.is_connected = self.is_connected
        response.state.is_powered_on = self.is_powered_on
        response.state.has_lease = self.has_lease
        response.state.current_strategy = self.current_strategy
        response.state.success_rate = self.success_rate
        return response
        
    def publish_state(self):
        """Publica estado atual da operação."""
        state = OperationState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.current_mode = self.current_mode
        state.is_connected = self.is_connected
        state.is_powered_on = self.is_powered_on
        state.has_lease = self.has_lease
        state.current_strategy = self.current_strategy
        state.success_rate = self.success_rate
        
        self.state_publisher.publish(state)
        
    def set_mode(self, mode: str):
        """Define o modo atual de operação."""
        self.current_mode = mode
        
    def set_strategy(self, strategy: str):
        """Define a estratégia atual."""
        self.current_strategy = strategy
        
    def update_success_rate(self, rate: float):
        """Atualiza a taxa de sucesso."""
        self.success_rate = rate


def main(args=None):
    rclpy.init(args=args)
    node = OperationStateManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 