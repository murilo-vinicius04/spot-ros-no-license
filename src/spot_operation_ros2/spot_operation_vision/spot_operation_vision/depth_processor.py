#!/usr/bin/env python3
"""
Nó para calcular a pose 3D dos objetos detectados usando imagem de profundidade.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, CameraInfo
from spot_operation_msgs.msg import ObjectDetection
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
import message_filters

class DepthProcessorNode(Node):
    def __init__(self):
        super().__init__('depth_processor')
        self.get_logger().info('Inicializando DepthProcessorNode...')

        # Parâmetros
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/aligned_depth_to_color/camera_info')
        self.declare_parameter('detections_topic', 'object_detections')
        self.declare_parameter('detections_3d_topic', 'object_detections_3d')
        self.declare_parameter('depth_window', 5)

        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        detections_topic = self.get_parameter('detections_topic').get_parameter_value().string_value
        detections_3d_topic = self.get_parameter('detections_3d_topic').get_parameter_value().string_value
        self.depth_window = self.get_parameter('depth_window').get_parameter_value().integer_value

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.fx = self.fy = self.cx = self.cy = None

        # Assina CameraInfo
        self.create_subscription(CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        # Assina detecções e profundidade (sincronizado)
        self.detection_sub = message_filters.Subscriber(self, ObjectDetection, detections_topic)
        self.depth_sub = message_filters.Subscriber(self, Image, depth_topic)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.detection_sub, self.depth_sub], 10, 0.2)
        self.ts.registerCallback(self.synced_callback)

        # Publisher para detecções 3D
        self.detection3d_pub = self.create_publisher(ObjectDetection, detections_3d_topic, 10)

    def camera_info_callback(self, msg):
        if self.camera_matrix is not None:
            return
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.fx = self.camera_matrix[0, 0]
        self.fy = self.camera_matrix[1, 1]
        self.cx = self.camera_matrix[0, 2]
        self.cy = self.camera_matrix[1, 2]
        self.get_logger().info('Camera intrinsics recebidos.')

    def synced_callback(self, detection_msg, depth_msg):
        if self.camera_matrix is None:
            self.get_logger().warn('Aguardando intrínsecos da câmera...')
            return
        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Erro ao converter depth: {e}')
            return
        # Centro do bounding box
        cx = int((detection_msg.x_min + detection_msg.x_max) / 2)
        cy = int((detection_msg.y_min + detection_msg.y_max) / 2)
        # Janela para média
        w = self.depth_window
        x1 = max(0, cx - w//2)
        x2 = min(depth_image.shape[1], cx + w//2 + 1)
        y1 = max(0, cy - w//2)
        y2 = min(depth_image.shape[0], cy + w//2 + 1)
        window = depth_image[y1:y2, x1:x2]
        # Remove zeros e outliers
        valid = window[(window > 0) & (window < 10000)]
        if valid.size == 0:
            self.get_logger().warn('Sem profundidade válida para o objeto.')
            return
        depth = float(np.median(valid)) / 1000.0  # mm -> m
        # Deprojeção
        x_cam = (cx - self.cx) * depth / self.fx
        y_cam = (cy - self.cy) * depth / self.fy
        z_cam = depth
        # Preenche pose_3d
        pose = PoseStamped()
        pose.header = detection_msg.header
        pose.pose.position.x = x_cam
        pose.pose.position.y = y_cam
        pose.pose.position.z = z_cam
        pose.pose.orientation.w = 1.0
        # Publica detecção 3D
        det3d = ObjectDetection()
        det3d.header = detection_msg.header
        det3d.class_name = detection_msg.class_name
        det3d.confidence = detection_msg.confidence
        det3d.x_min = detection_msg.x_min
        det3d.y_min = detection_msg.y_min
        det3d.x_max = detection_msg.x_max
        det3d.y_max = detection_msg.y_max
        det3d.pose_3d = pose
        self.detection3d_pub.publish(det3d)
        self.get_logger().debug(f'Objeto 3D: {det3d.class_name} pos=({x_cam:.2f},{y_cam:.2f},{z_cam:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 