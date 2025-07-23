#!/usr/bin/env python3
"""
Cliente de contagem de dedos para detecção de gestos.
Baseado no código original FingerCountClient.
Integração real com serviço de contagem de dedos.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_srvs.srv import Trigger
from spot_operation_msgs.msg import GestureCommand
from rclpy.timer import Timer
import time
from typing import Optional

class FingerCountClientNode(Node):
    """Calls /finger_count_node/get_finger_count until it gets 1 or 2."""
    
    def __init__(self):
        super().__init__('finger_count_client')
        self.get_logger().info('Inicializando FingerCountClientNode...')
        
        # Parâmetros
        self.declare_parameter('service_name', '/finger_count_node/get_finger_count')
        self.declare_parameter('check_interval', 0.5)  # segundos
        self.declare_parameter('publish_topic', 'gesture_command')
        self.declare_parameter('use_real_service', True)
        
        service_name = self.get_parameter('service_name').get_parameter_value().string_value
        check_interval = self.get_parameter('check_interval').get_parameter_value().double_value
        publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        use_real_service = self.get_parameter('use_real_service').get_parameter_value().bool_value
        
        # Estado
        self.current_mode = "unknown"
        self.last_finger_count = None
        self.mode_selection_active = False
        self.use_real_service = use_real_service
        
        # Publisher para comandos de gesto
        self.gesture_pub = self.create_publisher(GestureCommand, publish_topic, 10)
        
        # Timer para verificação periódica
        self.check_timer = self.create_timer(check_interval, self.check_mode_update)
        
        # Cliente de serviço (INTEGRAÇÃO REAL)
        if self.use_real_service:
            self.get_logger().info(f"Aguardando serviço real: {service_name}")
            self.service_client = self.create_client(Trigger, service_name)
        else:
            self.get_logger().info("Usando serviço simulado")
            self.service_client = None
        
        # Inicializa modo
        self.request_initial_mode()
        
    def request_initial_mode(self):
        """Solicita modo inicial (bloqueante)."""
        self.get_logger().info("Solicitando modo inicial...")
        
        if self.use_real_service:
            # INTEGRAÇÃO REAL COM SERVIÇO
            while not self.service_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("Serviço de contagem de dedos não disponível. Aguardando...")
                time.sleep(1.0)
                
            mode_code = self.request_mode()
            self.current_mode = "manual" if mode_code == 1 else "semi"
        else:
            # MODO SIMULADO
            self.current_mode = "manual"
            
        self.get_logger().info(f"🚀 Modo de operação selecionado: {self.current_mode.upper()}")
        
        # Publica comando inicial
        self.publish_gesture_command()
        
    def request_mode(self) -> int:
        """Blocks until receiving 1 (manual) or 2 (semi-autonomous) - INTEGRAÇÃO REAL."""
        while not rclpy.ok():
            try:
                request = Trigger.Request()
                future = self.service_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.done():
                    response = future.result()
                    if response.success and response.message in ("1", "2"):
                        return int(response.message)
                        
                self.get_logger().info(f"FingerCount retornou '{response.message}'. Tentando novamente...")
                time.sleep(0.3)
                
            except Exception as e:
                self.get_logger().warn(f"FingerCount service failed: {e}")
                time.sleep(0.5)
                
    def check_mode_update(self):
        """Checa uma vez se o número de dedos mudou e atualiza o modo se necessário."""
        if self.use_real_service:
            # INTEGRAÇÃO REAL COM SERVIÇO
            if not self.service_client.service_is_ready():
                return
                
            try:
                request = Trigger.Request()
                future = self.service_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
                
                if not future.done():
                    return
                    
                response = future.result()
                if not response.success or response.message not in ("1", "2", "3"):
                    return
                    
                code = int(response.message)
                expected = 1 if self.current_mode == "manual" else 2
                
                if code == expected:
                    return  # Nada mudou
                elif code in (1, 2):
                    # Mudança de modo
                    novo = "manual" if code == 1 else "semi"
                    self.get_logger().info(f"🔁 Modo alterado dinamicamente para: {novo.upper()}")
                    self.current_mode = novo
                    self.publish_gesture_command()
                elif code == 3:
                    # Gesto de modo detectado
                    self.get_logger().info("👆 Gesto de MODO detectado → solicitando novo modo")
                    self.mode_selection_active = True
                    self.request_mode_selection()
                    
            except Exception as e:
                self.get_logger().warn(f"Erro ao consultar modo de dedo: {e}")
        else:
            # MODO SIMULADO - mantém modo atual
            pass
            
    def request_mode_selection(self):
        """Solicita seleção de modo quando detecta 3 dedos - INTEGRAÇÃO REAL."""
        if not self.use_real_service:
            return
            
        while not rclpy.ok() and self.mode_selection_active:
            try:
                request = Trigger.Request()
                future = self.service_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.done():
                    response = future.result()
                    if response.success and response.message in ("1", "2"):
                        novo = "manual" if response.message == "1" else "semi"
                        self.get_logger().info(f"✅ Novo modo definido: {novo.upper()}")
                        self.current_mode = novo
                        self.mode_selection_active = False
                        self.publish_gesture_command()
                        return
                        
                time.sleep(0.5)
                
            except Exception as e:
                self.get_logger().warn(f"Erro na seleção de modo: {e}")
                time.sleep(0.5)
                
    def publish_gesture_command(self):
        """Publica comando de gesto atual."""
        gesture_msg = GestureCommand()
        gesture_msg.header.stamp = self.get_clock().now().to_msg()
        
        if self.current_mode == "manual":
            gesture_msg.gesture_type = 2  # modo_manual
            gesture_msg.finger_count = 1
        elif self.current_mode == "semi":
            gesture_msg.gesture_type = 3  # modo_autonomo
            gesture_msg.finger_count = 2
        else:
            gesture_msg.gesture_type = 0  # desconhecido
            gesture_msg.finger_count = 0
            
        gesture_msg.is_valid = True
        self.gesture_pub.publish(gesture_msg)
        
        self.get_logger().debug(f"Comando de gesto publicado: {self.current_mode}")


def main(args=None):
    rclpy.init(args=args)
    node = FingerCountClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 