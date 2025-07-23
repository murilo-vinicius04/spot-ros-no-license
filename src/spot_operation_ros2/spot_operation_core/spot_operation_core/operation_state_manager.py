#!/usr/bin/env python3
"""
Gerenciador de estado da operação do Spot.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time
from typing import Dict, Any, Optional
from enum import Enum

from spot_operation_msgs.msg import OperationState
from spot_operation_msgs.srv import GetOperationState


class OperationMode(Enum):
    """Modos de operação disponíveis."""
    UNKNOWN = "unknown"
    MANUAL = "manual"
    AUTONOMOUS = "autonomous"
    MANIPULATION = "manipulation"
    CALIBRATION = "calibration"
    ERROR = "error"


class OperationStateManager(Node):
    """Gerencia o estado da operação do Spot."""
    
    def __init__(self):
        super().__init__('operation_state_manager')
        
        # Parâmetros
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('default_mode', 'manual')
        
        self.publish_rate = self.get_parameter('publish_rate').value
        self.default_mode = self.get_parameter('default_mode').value
        
        # Estado atual
        self.current_mode = OperationMode(self.default_mode)
        self.is_connected = False
        self.is_powered_on = False
        self.has_lease = False
        self.current_strategy = "none"
        self.success_rate = 0.0
        self.error_message = ""
        
        # Estatísticas
        self.operation_stats = {
            'total_operations': 0,
            'successful_operations': 0,
            'failed_operations': 0,
            'last_operation_time': None,
            'operation_history': []
        }
        
        # Publishers e Services
        self.state_publisher = self.create_publisher(
            OperationState, 'operation_state', 10
        )
        self.state_service = self.create_service(
            GetOperationState, 'get_operation_state', self.get_operation_state_callback
        )
        
        # Timer para publicação do estado
        self.state_timer = self.create_timer(1.0 / self.publish_rate, self.publish_state)
        
        self.get_logger().info("OperationStateManager inicializado")
        
    def set_mode(self, mode: OperationMode):
        """Define o modo de operação atual."""
        old_mode = self.current_mode
        self.current_mode = mode
        self.get_logger().info(f"Modo alterado: {old_mode.value} -> {mode.value}")
        
    def set_connection_status(self, is_connected: bool):
        """Define o status de conexão."""
        self.is_connected = is_connected
        
    def set_power_status(self, is_powered_on: bool):
        """Define o status de energia."""
        self.is_powered_on = is_powered_on
        
    def set_lease_status(self, has_lease: bool):
        """Define o status do lease."""
        self.has_lease = has_lease
        
    def set_current_strategy(self, strategy: str):
        """Define a estratégia atual."""
        self.current_strategy = strategy
        
    def set_error_message(self, error_message: str):
        """Define a mensagem de erro atual."""
        self.error_message = error_message
        if error_message:
            self.set_mode(OperationMode.ERROR)
        
    def clear_error(self):
        """Limpa o erro e retorna ao modo anterior."""
        self.error_message = ""
        if self.current_mode == OperationMode.ERROR:
            self.set_mode(OperationMode.MANUAL)
        
    def record_operation(self, success: bool, operation_type: str = "unknown"):
        """Registra uma operação para estatísticas."""
        self.operation_stats['total_operations'] += 1
        self.operation_stats['last_operation_time'] = time.time()
        
        if success:
            self.operation_stats['successful_operations'] += 1
        else:
            self.operation_stats['failed_operations'] += 1
            
        # Adiciona à história
        operation_record = {
            'timestamp': time.time(),
            'type': operation_type,
            'success': success
        }
        self.operation_stats['operation_history'].append(operation_record)
        
        # Mantém apenas os últimos 100 registros
        if len(self.operation_stats['operation_history']) > 100:
            self.operation_stats['operation_history'] = self.operation_stats['operation_history'][-100:]
        
        # Calcula taxa de sucesso
        if self.operation_stats['total_operations'] > 0:
            self.success_rate = (self.operation_stats['successful_operations'] / 
                               self.operation_stats['total_operations'])
        
    def get_operation_state_callback(self, request, response):
        """Callback para o serviço de obtenção do estado da operação."""
        state = self._create_operation_state()
        response.state = state
        return response
        
    def publish_state(self):
        """Publica o estado atual da operação."""
        state = self._create_operation_state()
        self.state_publisher.publish(state)
        
    def _create_operation_state(self) -> OperationState:
        """Cria uma mensagem de estado da operação."""
        state = OperationState()
        state.header.stamp = self.get_clock().now().to_msg()
        state.current_mode = self.current_mode.value
        state.is_connected = self.is_connected
        state.is_powered_on = self.is_powered_on
        state.has_lease = self.has_lease
        state.current_strategy = self.current_strategy
        state.success_rate = self.success_rate
        
        return state
        
    def get_statistics(self) -> Dict[str, Any]:
        """Retorna estatísticas da operação."""
        return {
            'mode': self.current_mode.value,
            'is_connected': self.is_connected,
            'is_powered_on': self.is_powered_on,
            'has_lease': self.has_lease,
            'current_strategy': self.current_strategy,
            'success_rate': self.success_rate,
            'total_operations': self.operation_stats['total_operations'],
            'successful_operations': self.operation_stats['successful_operations'],
            'failed_operations': self.operation_stats['failed_operations'],
            'last_operation_time': self.operation_stats['last_operation_time'],
            'error_message': self.error_message
        }
        
    def reset_statistics(self):
        """Reseta as estatísticas da operação."""
        self.operation_stats = {
            'total_operations': 0,
            'successful_operations': 0,
            'failed_operations': 0,
            'last_operation_time': None,
            'operation_history': []
        }
        self.success_rate = 0.0
        self.get_logger().info("Estatísticas resetadas")
        
    def is_ready_for_operation(self) -> bool:
        """Verifica se o sistema está pronto para operação."""
        return (self.is_connected and 
                self.is_powered_on and 
                self.has_lease and 
                self.current_mode != OperationMode.ERROR)
        
    def get_health_status(self) -> Dict[str, Any]:
        """Retorna o status de saúde do sistema."""
        return {
            'ready_for_operation': self.is_ready_for_operation(),
            'connection_ok': self.is_connected,
            'power_ok': self.is_powered_on,
            'lease_ok': self.has_lease,
            'no_errors': self.current_mode != OperationMode.ERROR,
            'error_message': self.error_message
        }


def main(args=None):
    rclpy.init(args=args)
    state_manager = OperationStateManager()
    
    try:
        rclpy.spin(state_manager)
    except KeyboardInterrupt:
        pass
    finally:
        state_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 