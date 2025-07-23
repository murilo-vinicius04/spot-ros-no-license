#!/usr/bin/env python3
"""
Nó de detecção de objetos usando YOLO para ROS2.
Usando Spot Driver ROS2.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from spot_operation_msgs.msg import ObjectDetection
from cv_bridge import CvBridge
import cv2
import numpy as np
import pandas as pd
from ultralytics import YOLO
import os
import math

# Spot Driver ROS2 imports
from sensor_msgs.msg import Image, CameraInfo
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.get_logger().info('Inicializando ObjectDetectorNode...')

        # Parâmetros
        self.declare_parameter('model_path', 'models/yolo11n.pt')
        self.declare_parameter('allowed_objects_csv', 'config/allowed_objects.csv')
        self.declare_parameter('confidence_threshold', 0.3)
        self.declare_parameter('publish_topic', 'object_detections')
        self.declare_parameter('use_spot_cameras', True)
        self.declare_parameter('rgb_topic', '/hand_color_image')
        self.declare_parameter('depth_topic', '/hand_depth_in_hand_color_frame')
        self.declare_parameter('camera_info_topic', '/hand_color_image_info')

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        allowed_objects_csv = self.get_parameter('allowed_objects_csv').get_parameter_value().string_value
        confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        use_spot_cameras = self.get_parameter('use_spot_cameras').get_parameter_value().bool_value
        rgb_topic = self.get_parameter('rgb_topic').get_parameter_value().string_value
        depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value

        # Inicializa YOLO
        if not os.path.isfile(model_path):
            self.get_logger().error(f"Modelo YOLO não encontrado: {model_path}")
            raise FileNotFoundError(model_path)
        self.model = YOLO(model_path)
        self.get_logger().info(f"Modelo YOLO carregado: {model_path}")

        # Lê CSV de objetos permitidos
        if not os.path.isfile(allowed_objects_csv):
            self.get_logger().error(f"CSV de objetos permitidos não encontrado: {allowed_objects_csv}")
            raise FileNotFoundError(allowed_objects_csv)
        self.allowed_objects = pd.read_csv(allowed_objects_csv)
        self.allowed_objects['class'] = self.allowed_objects['class'].str.strip().str.lower()
        self.get_logger().info(f"Objetos permitidos carregados: {allowed_objects_csv}")

        self.confidence_threshold = confidence_threshold
        self.bridge = CvBridge()
        self.use_spot_cameras = use_spot_cameras

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publisher para detecções
        self.detection_pub = self.create_publisher(ObjectDetection, publish_topic, 10)

        # Estado das imagens
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.latest_camera_info = None
        self.image_sync_tolerance = 0.1  # segundos

        if self.use_spot_cameras:
            # SPOT DRIVER ROS2 - Subscribers
            self.rgb_sub = self.create_subscription(
                Image, rgb_topic, self.rgb_callback, 10
            )
            self.depth_sub = self.create_subscription(
                Image, depth_topic, self.depth_callback, 10
            )
            self.camera_info_sub = self.create_subscription(
                CameraInfo, camera_info_topic, self.camera_info_callback, 10
            )
            
            # Timer para processamento sincronizado
            self.process_timer = self.create_timer(1.0, self.process_synchronized_images)
        else:
            # Subscriber de imagem ROS2 genérico
            self.image_sub = self.create_subscription(
                Image, '/camera/color/image_raw', self.image_callback, 10)

    def rgb_callback(self, msg: Image):
        """Callback para imagem RGB do Spot Driver."""
        self.latest_rgb_image = msg

    def depth_callback(self, msg: Image):
        """Callback para imagem de profundidade do Spot Driver."""
        self.latest_depth_image = msg

    def camera_info_callback(self, msg: CameraInfo):
        """Callback para informações da câmera do Spot Driver."""
        self.latest_camera_info = msg

    def process_synchronized_images(self):
        """Processa imagens RGB e depth sincronizadas do Spot Driver."""
        if (self.latest_rgb_image is None or 
            self.latest_depth_image is None or 
            self.latest_camera_info is None):
            return

        # Verifica se as imagens estão sincronizadas
        rgb_time = self.latest_rgb_image.header.stamp.sec + self.latest_rgb_image.header.stamp.nanosec * 1e-9
        depth_time = self.latest_depth_image.header.stamp.sec + self.latest_depth_image.header.stamp.nanosec * 1e-9
        
        if abs(rgb_time - depth_time) > self.image_sync_tolerance:
            self.get_logger().warn("Imagens RGB e depth não sincronizadas")
            return

        try:
            # Converte imagens
            rgb_cv = self.bridge.imgmsg_to_cv2(self.latest_rgb_image, desired_encoding='bgr8')
            depth_cv = self.bridge.imgmsg_to_cv2(self.latest_depth_image, desired_encoding='16UC1')
            
            # Executa detecção YOLO com profundidade
            self._yolo_depth_deproject(rgb_cv, depth_cv, self.latest_camera_info)
            
        except Exception as e:
            self.get_logger().error(f"Erro ao processar imagens: {e}")

    def _yolo_depth_deproject(self, rgb_img, depth_img, camera_info):
        """Detecta objetos e calcula poses 3D usando Spot Driver ROS2."""
        try:
            # Detecta objetos na imagem RGB
            results = self.model(cv2.cvtColor(rgb_img, cv2.COLOR_BGR2RGB))
            boxes = results[0].boxes.xyxy.cpu().numpy()
            classes = results[0].boxes.cls.cpu().numpy().astype(int)
            names = [results[0].names[c].lower() for c in classes]
            
            # Filtra objetos permitidos
            candidates = self._filter_allowed_objects(boxes, names)
            
            if not candidates:
                return None
                
            # Seleciona o melhor candidato
            _, _, idx = candidates[0]
            x1, y1, x2, y2 = boxes[idx]
            cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
            
            # Obtém profundidade no centro do objeto
            depth_rows, depth_cols = depth_img.shape
            cx_safe = min(max(0, int(cx)), depth_cols - 1)
            cy_safe = min(max(0, int(cy)), depth_rows - 1)
            
            raw_mm = int(depth_img[cy_safe, cx_safe])
            
            # Filtra profundidade
            min_depth_mm = 300   # 30 cm
            max_depth_mm = 3000  # 3 m
            
            depth_mm = raw_mm if (min_depth_mm <= raw_mm <= max_depth_mm) else min_depth_mm
            depth_m = depth_mm / 1000.0
            
            # De-projeção do pixel -> quadro de câmera usando camera_info
            fx = camera_info.k[0]  # focal length x
            fy = camera_info.k[4]  # focal length y
            cx0 = camera_info.k[2]  # principal point x
            cy0 = camera_info.k[5]  # principal point y
            
            x_cam = (cx - cx0) * depth_m / fx
            y_cam = (cy - cy0) * depth_m / fy
            z_cam = depth_m
            
            # Quadro de câmera → quadro de odom
            try:
                transform = self.tf_buffer.lookup_transform(
                    'odom', camera_info.header.frame_id,
                    rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0)
                )
                
                # Cria pose do objeto no quadro da câmera
                pose_cam = PoseStamped()
                pose_cam.header.frame_id = camera_info.header.frame_id
                pose_cam.header.stamp = self.get_clock().now().to_msg()
                pose_cam.pose.position.x = x_cam
                pose_cam.pose.position.y = y_cam
                pose_cam.pose.position.z = z_cam
                pose_cam.pose.orientation.w = 1.0
                
                # Transforma para odom
                pose_odom = tf2_geometry_msgs.do_transform_pose(pose_cam, transform)
                
            except Exception as e:
                self.get_logger().warn(f"Falha ao transformar pose: {e}")
                return None
            
            # Publica detecção 3D
            det_msg = ObjectDetection()
            det_msg.header.stamp = self.get_clock().now().to_msg()
            det_msg.header.frame_id = "odom"
            det_msg.class_name = names[idx]
            det_msg.confidence = float(results[0].boxes.conf.cpu().numpy()[idx])
            det_msg.x_min = float(x1)
            det_msg.y_min = float(y1)
            det_msg.x_max = float(x2)
            det_msg.y_max = float(y2)
            det_msg.pose_3d = pose_odom
            
            self.detection_pub.publish(det_msg)
            
            self.get_logger().debug(
                f"Objeto 3D detectado: {names[idx]} em ({pose_odom.pose.position.x:.3f}, {pose_odom.pose.position.y:.3f}, {pose_odom.pose.position.z:.3f})"
            )
            
            return np.array([pose_odom.pose.position.x, pose_odom.pose.position.y, pose_odom.pose.position.z])
            
        except Exception as e:
            self.get_logger().error(f"Erro na de-projeção YOLO: {e}")
            return None

    def _filter_allowed_objects(self, boxes, names):
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

    def image_callback(self, msg: Image):
        """Callback para imagens ROS2 genéricas (quando não usa câmeras do Spot)."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Erro ao converter imagem: {e}")
            return

        results = self.model(cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB))
        boxes = results[0].boxes.xyxy.cpu().numpy()
        scores = results[0].boxes.conf.cpu().numpy()
        classes = results[0].boxes.cls.cpu().numpy().astype(int)
        names = [results[0].names[c].lower() for c in classes]

        for i, (box, score, name) in enumerate(zip(boxes, scores, names)):
            if score < self.confidence_threshold:
                continue
            if name not in self.allowed_objects['class'].values:
                continue
            # Busca prioridade
            priority = int(self.allowed_objects[self.allowed_objects['class'] == name]['priority'].iloc[0])
            # Monta mensagem
            det_msg = ObjectDetection()
            det_msg.header = msg.header if msg.header else Header()
            det_msg.class_name = name
            det_msg.confidence = float(score)
            det_msg.x_min = float(box[0])
            det_msg.y_min = float(box[1])
            det_msg.x_max = float(box[2])
            det_msg.y_max = float(box[3])
            # pose_3d pode ser preenchido por outro nó (ex: depth_processor)
            self.detection_pub.publish(det_msg)
            self.get_logger().debug(f"Objeto detectado: {name} (conf={score:.2f}, prioridade={priority})")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 