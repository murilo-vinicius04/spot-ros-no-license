#!/usr/bin/env python3
"""
Gerenciador de estratégias de grasp para o Spot.
Usando Spot Driver ROS2.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from spot_operation_msgs.msg import ObjectDetection, GraspResult
from spot_operation_msgs.srv import ExecuteGrasp, SetGraspStrategy
from spot_operation_msgs.action import GraspObject
from geometry_msgs.msg import PoseStamped
import numpy as np
from abc import ABC, abstractmethod
from typing import Dict, List, Optional
import time

# Spot Driver ROS2 imports
from spot_msgs.action import Manipulation
from spot_msgs.msg import ManipulationState
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class GraspStrategyBase(ABC):
    """Interface base para estratégias de grasp."""
    
    def __init__(self, name: str):
        self.name = name
        self.success_count = 0
        self.attempt_count = 0
        self.node = None  # Será definido pelo GraspManager
        
    @abstractmethod
    def execute(self, detection: ObjectDetection) -> bool:
        """
        Executa a estratégia de grasp.
        
        Args:
            detection: Detecção do objeto a ser agarrado
            
        Returns:
            bool: True se o grasp foi bem-sucedido, False caso contrário
        """
        pass
        
    def get_success_rate(self) -> float:
        """Retorna a taxa de sucesso da estratégia."""
        if self.attempt_count == 0:
            return 0.0
        return self.success_count / self.attempt_count


class YOLOGraspStrategy(GraspStrategyBase):
    """Estratégia de grasp baseada em detecção YOLO usando Spot Driver ROS2."""
    
    def __init__(self, name: str = "yolo_grasp", image_topic: str = "/hand_color_image"):
        super().__init__(name)
        self.image_topic = image_topic
        self.bridge = CvBridge()
        self.latest_image = None
        
    def execute(self, detection: ObjectDetection) -> bool:
        """Executa grasp com base na detecção YOLO usando Spot Driver ROS2."""
        if self.node is None:
            return False
            
        self.node.get_logger().info(f"Executando grasp YOLO para {detection.class_name}")
        
        try:
            # Calcula centro do objeto na imagem
            cx = int((detection.x_min + detection.x_max) / 2)
            cy = int((detection.y_min + detection.y_max) / 2)
            
            # Executa grasp via Spot Driver ROS2 Action
            goal = Manipulation.Goal()
            goal.manipulation_request.pick_object_in_image.pixel_xy.x = cx
            goal.manipulation_request.pick_object_in_image.pixel_xy.y = cy
            
            # Envia goal para action server
            if hasattr(self.node, 'manipulation_client'):
                future = self.node.manipulation_client.send_goal_async(goal)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
                
                if future.done():
                    goal_handle = future.result()
                    if goal_handle.accepted:
                        # Monitora resultado
                        result = self._monitor_manipulation_result(goal_handle)
                        return result
                    else:
                        self.node.get_logger().error("Goal rejeitado pelo action server")
                        return False
                else:
                    self.node.get_logger().error("Timeout ao enviar goal")
                    return False
            else:
                self.node.get_logger().error("Manipulation client não disponível")
                return False
                
        except Exception as e:
            self.node.get_logger().error(f"Erro na execução do grasp YOLO: {e}")
            return False
        
    def _monitor_manipulation_result(self, goal_handle, poll_interval=0.2):
        """Monitora o resultado da manipulação via Spot Driver ROS2."""
        while True:
            try:
                future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=poll_interval)
                
                if future.done():
                    result = future.result().result
                    if result.success:
                        self.node.get_logger().info("✅ Grasp executado com sucesso")
                        return True
                    else:
                        self.node.get_logger().warn("❌ Grasp falhou")
                        return False
                        
            except Exception as e:
                self.node.get_logger().error(f"Erro ao monitorar resultado: {e}")
                return False


class GraspManagerNode(Node):
    """Gerencia diferentes estratégias de grasp usando Spot Driver ROS2."""
    
    def __init__(self):
        super().__init__('grasp_manager')
        self.get_logger().info('Inicializando GraspManagerNode...')
        
        # Parâmetros
        self.declare_parameter('default_strategy', 'front_yolo')
        self.declare_parameter('fallback_strategy', 'side_yolo')
        self.declare_parameter('detections_topic', 'object_detections_3d')
        self.declare_parameter('results_topic', 'grasp_results')
        
        self.default_strategy = self.get_parameter('default_strategy').get_parameter_value().string_value
        self.fallback_strategy = self.get_parameter('fallback_strategy').get_parameter_value().string_value
        detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        results_topic = self.get_parameter('results_topic').get_parameter_value().string_value
        
        # Estratégias
        self.strategies: Dict[str, GraspStrategyBase] = {}
        self.current_strategy = self.default_strategy
        self.last_used_strategy = None
        
        # SPOT DRIVER ROS2 - Action Client
        self.manipulation_client = ActionClient(self, Manipulation, '/manipulation')
        
        # Publishers e Subscribers
        self.detection_sub = self.create_subscription(
            ObjectDetection, detections_topic, self.detection_callback, 10
        )
        self.results_pub = self.create_publisher(GraspResult, results_topic, 10)
        
        # Services
        self.execute_grasp_service = self.create_service(
            ExecuteGrasp, 'execute_grasp', self.execute_grasp_callback
        )
        self.set_strategy_service = self.create_service(
            SetGraspStrategy, 'set_grasp_strategy', self.set_strategy_callback
        )
        
        # Action Server
        self.action_server = ActionServer(
            self, GraspObject, 'grasp_object',
            self.grasp_action_callback,
            callback_group=ReentrantCallbackGroup()
        )
        
        # Inicializa estratégias
        self._initialize_strategies()
        
    def _initialize_strategies(self):
        """Inicializa as estratégias de grasp disponíveis."""
        # Estratégia YOLO frontal (Spot Driver ROS2)
        front_yolo_strategy = YOLOGraspStrategy("front_yolo", "/hand_color_image")
        front_yolo_strategy.node = self
        self.strategies["front_yolo"] = front_yolo_strategy
        
        # Estratégia YOLO lateral (Spot Driver ROS2)
        side_yolo_strategy = YOLOGraspStrategy("side_yolo", "/frontleft_fisheye_image")
        side_yolo_strategy.node = self
        self.strategies["side_yolo"] = side_yolo_strategy
        
        self.get_logger().info(f"Estratégias inicializadas: {list(self.strategies.keys())}")
        
    def detection_callback(self, msg: ObjectDetection):
        """Callback para detecções de objetos."""
        # Aqui você pode implementar lógica automática de grasp
        # Por exemplo, se detectar um objeto com alta confiança, executar grasp
        if msg.confidence > 0.8:
            self.get_logger().info(f"Objeto detectado com alta confiança: {msg.class_name}")
            # self.execute_grasp(msg, self.current_strategy)
            
    def execute_grasp(self, detection: ObjectDetection, strategy_name: str) -> bool:
        """
        Executa a estratégia de grasp especificada usando Spot Driver ROS2.
        
        Args:
            detection: Detecção do objeto
            strategy_name: Nome da estratégia a ser executada
            
        Returns:
            bool: True se o grasp foi bem-sucedido, False caso contrário
        """
        if strategy_name not in self.strategies:
            self.get_logger().error(f"Estratégia '{strategy_name}' não encontrada.")
            return False
            
        strategy = self.strategies[strategy_name]
        self.last_used_strategy = strategy_name
        
        # Executa a estratégia (Spot Driver ROS2)
        result = strategy.execute(detection)
        
        # Publica resultado
        grasp_result = GraspResult()
        grasp_result.success = result
        grasp_result.strategy_used = strategy_name
        grasp_result.error_message = "" if result else "Grasp falhou"
        grasp_result.confidence = detection.confidence
        grasp_result.object_pose = detection.pose_3d
        
        self.results_pub.publish(grasp_result)
        
        # Loga estatísticas
        success_rate = strategy.get_success_rate() * 100
        self.get_logger().info(
            f"Estratégia '{strategy_name}' - Taxa de sucesso: {success_rate:.1f}% "
            f"({strategy.success_count}/{strategy.attempt_count})"
        )
        
        return result
        
    def execute_grasp_callback(self, request, response):
        """Callback para o serviço ExecuteGrasp."""
        if not request.target_pose:
            response.success = False
            response.message = "Pose alvo não fornecida"
            return response
            
        # Cria detecção simulada
        detection = ObjectDetection()
        detection.class_name = "unknown"
        detection.confidence = 1.0
        detection.pose_3d = request.target_pose
        
        # Executa grasp
        success = self.execute_grasp(detection, request.strategy_name)
        
        response.success = success
        response.message = "Grasp executado com sucesso" if success else "Grasp falhou"
        
        return response
        
    def set_strategy_callback(self, request, response):
        """Callback para o serviço SetGraspStrategy."""
        if request.strategy_name in self.strategies:
            self.current_strategy = request.strategy_name
            response.success = True
            response.message = f"Estratégia alterada para: {request.strategy_name}"
        else:
            response.success = False
            response.message = f"Estratégia '{request.strategy_name}' não encontrada"
            
        return response
        
    def grasp_action_callback(self, goal_handle):
        """Callback para o Action Server GraspObject."""
        goal = goal_handle.request
        
        # Cria detecção a partir do goal
        detection = ObjectDetection()
        detection.class_name = goal.object_class
        detection.confidence = goal.confidence
        detection.pose_3d = goal.target_pose
        
        # Executa grasp
        success = self.execute_grasp(detection, goal.strategy_name)
        
        # Retorna resultado
        result = GraspObject.Result()
        result.success = success
        result.strategy_used = goal.strategy_name
        result.error_message = "" if success else "Grasp falhou"
        
        goal_handle.succeed()
        return result
        
    def get_success_rates(self) -> Dict[str, float]:
        """Retorna as taxas de sucesso de todas as estratégias registradas."""
        rates = {}
        for name, strategy in self.strategies.items():
            rates[name] = strategy.get_success_rate()
        return rates


def main(args=None):
    rclpy.init(args=args)
    node = GraspManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 