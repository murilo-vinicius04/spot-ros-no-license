#!/usr/bin/env python3
"""
Controlador principal do Spot com suporte a campos de potencial e detecção de colisão.
Baseado no código original SpotController.
Usando Spot Driver ROS2.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.timer import Timer
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from sensor_msgs.msg import JointState, Image, CameraInfo
from std_msgs.msg import Int32
from spot_operation_msgs.msg import GestureCommand, ObjectDetection, GraspResult
from spot_operation_msgs.srv import GetOperationState
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import copy
import time
import math
from typing import Optional, Tuple
from enum import Enum

# Spot Driver ROS2 imports
from std_srvs.srv import Trigger
from spot_msgs.msg import ManipulationState
from spot_msgs.action import RobotCommand, Manipulation
from rclpy.action import ActionClient

# YOLO imports
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import pandas as pd

class ControlMode(Enum):
    """Modos de controle disponíveis."""
    MANUAL = "manual"
    AUTONOMOUS = "autonomous"
    MANIPULATION = "manipulation"

class SpotControllerNode(Node):
    """Controller with light potential field support in MANUAL mode using Spot Driver ROS2."""
    
    def __init__(self):
        super().__init__('spot_controller')
        self.get_logger().info('Inicializando SpotControllerNode...')
        
        # Parâmetros
        self.declare_parameter('group_name', 'manipulator')
        self.declare_parameter('end_effector_link', 'arm_link_fngr')
        self.declare_parameter('control_rate', 5.0)  # Hz
        self.declare_parameter('pf_k_att', 0.2)
        self.declare_parameter('pf_attraction_distance', 0.4)  # metros
        self.declare_parameter('escape_distance', 0.2)  # metros
        self.declare_parameter('model_path', 'models/yolo11n.pt')
        self.declare_parameter('allowed_objects_csv', 'config/allowed_objects.csv')
        
        group_name = self.get_parameter('group_name').get_parameter_value().string_value
        end_effector_link = self.get_parameter('end_effector_link').get_parameter_value().string_value
        control_rate = self.get_parameter('control_rate').get_parameter_value().double_value
        self.pf_k_att = self.get_parameter('pf_k_att').get_parameter_value().double_value
        self.pf_attraction_distance = self.get_parameter('pf_attraction_distance').get_parameter_value().double_value
        self.escape_distance = self.get_parameter('escape_distance').get_parameter_value().double_value
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        allowed_objects_csv = self.get_parameter('allowed_objects_csv').get_parameter_value().string_value
        
        # Estado do controle
        self.control_mode = ControlMode.MANUAL
        self.current_gesture = 0
        self.manipulation_mode = False
        self.is_orientation_locked = False
        self.frozen_orientation = None
        self.current_hand_orientation = None  # Orientação da mão do hand_orientation_estimator
        
        # Campos de potencial
        self.pf_target_odom = None
        self.last_detection_time = 0.0
        self.target_valid_duration = 3.0
        
        # Detecção de colisão
        self.is_colliding = False
        self.backup_pose = None
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # MoveIt
        self.moveit_group = None
        self._initialize_moveit(group_name, end_effector_link)
        
        # YOLO Object Detector (como no código original)
        self._initialize_yolo_detector(model_path, allowed_objects_csv)
        
        # Bridge para imagens
        self.bridge = CvBridge()
        
        # SPOT DRIVER ROS2 - Publishers
        self.arm_joint_pub = self.create_publisher(
            JointState, '/arm_joint_commands', 10
        )
        
        # SPOT DRIVER ROS2 - Services
        self.open_gripper_client = self.create_client(Trigger, '/open_gripper')
        self.close_gripper_client = self.create_client(Trigger, '/close_gripper')
        self.arm_stow_client = self.create_client(Trigger, '/arm_stow')
        self.arm_unstow_client = self.create_client(Trigger, '/arm_unstow')
        
        # SPOT DRIVER ROS2 - Action Clients
        self.robot_command_client = ActionClient(self, RobotCommand, '/robot_command')
        self.manipulation_client = ActionClient(self, Manipulation, '/manipulation')
        
        # SPOT DRIVER ROS2 - Subscribers (para campos de potencial)
        self.rgb_image_sub = self.create_subscription(
            Image, '/hand_color_image', self.rgb_image_callback, 10
        )
        self.depth_image_sub = self.create_subscription(
            Image, '/hand_depth_in_hand_color_frame', self.depth_image_callback, 10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/hand_color_image_info', self.camera_info_callback, 10
        )
        self.manipulation_state_sub = self.create_subscription(
            ManipulationState, '/manipulation_state', self.manipulation_state_callback, 10
        )
        
        # ARM POSE ESTIMATOR - Subscribers
        self.hand_orientation_sub = self.create_subscription(
            PoseStamped, '/hand_orientation', self.hand_orientation_callback, 10
        )
        
        # Subscribers
        self.gesture_sub = self.create_subscription(
            GestureCommand, 'gesture_command', self.gesture_callback, 10
        )
        self.object_detection_sub = self.create_subscription(
            ObjectDetection, 'object_detections_3d', self.object_detection_callback, 10
        )
        self.grasp_result_sub = self.create_subscription(
            GraspResult, 'grasp_results', self.grasp_result_callback, 10
        )
        self.planning_scene_sub = self.create_subscription(
            moveit_msgs.msg.PlanningScene, '/move_group/monitored_planning_scene',
            self.planning_scene_callback, 10
        )
        
        # Services
        self.get_state_service = self.create_service(
            GetOperationState, 'get_control_state', self.get_control_state_callback
        )
        
        # Estado das imagens para campos de potencial
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.latest_camera_info = None
        
        # Timer de controle
        self.control_timer = self.create_timer(1.0 / control_rate, self.control_loop)
        
        self.get_logger().info("SpotControllerNode inicializado!")
        
    def _initialize_yolo_detector(self, model_path: str, allowed_objects_csv: str):
        """Inicializa detector YOLO como no código original."""
        try:
            self.model = YOLO(model_path)
            self.allowed_objects = pd.read_csv(allowed_objects_csv)
            self.allowed_objects['class'] = self.allowed_objects['class'].str.strip().str.lower()
            self.get_logger().info(f"Detector YOLO inicializado: {model_path}")
        except Exception as e:
            self.get_logger().error(f"Falha ao inicializar detector YOLO: {e}")
            self.model = None
            self.allowed_objects = None
        
    def _initialize_moveit(self, group_name: str, end_effector_link: str):
        """Inicializa MoveIt."""
        try:
            moveit_commander.roscpp_initialize([])
            self.moveit_group = moveit_commander.MoveGroupCommander(group_name)
            self.moveit_group.set_end_effector_link(end_effector_link)
            
            self.get_logger().info(
                f"MoveIt inicializado: End-effector ({end_effector_link}) em relação a: "
                f"{self.moveit_group.get_pose_reference_frame()}"
            )
            
            # Aplica restrições de juntas
            self.apply_wrist_lock()
            
        except Exception as e:
            self.get_logger().error(f"Falha ao inicializar MoveIt: {e}")
            self.moveit_group = None
            
    def apply_wrist_lock(self):
        """Aplica restrições nas juntas específicas para travar o punho."""
        if not self.moveit_group:
            return
            
        try:
            names = self.moveit_group.get_active_joints()
            vals = self.moveit_group.get_current_joint_values()
            
            # Trava as juntas do punho
            constraints = moveit_msgs.msg.Constraints()
            
            for joint_name in ["arm_wr0", "arm_wr1"]:
                if joint_name in names:
                    idx = names.index(joint_name)
                    locked_value = vals[idx]
                    self.get_logger().info(f"🔒 Travando {joint_name} em {locked_value:.3f} rad")
                    
                    constraint = moveit_msgs.msg.JointConstraint()
                    constraint.joint_name = joint_name
                    constraint.position = locked_value
                    constraint.tolerance_above = 0.0
                    constraint.tolerance_below = 0.0
                    constraint.weight = 1.0
                    constraints.joint_constraints.append(constraint)
            
            self.moveit_group.set_path_constraints(constraints)
            
        except Exception as e:
            self.get_logger().warn(f"Falha ao aplicar wrist lock: {e}")
            
    def rgb_image_callback(self, msg: Image):
        """Callback para imagem RGB do Spot Driver."""
        self.latest_rgb_image = msg
        
    def depth_image_callback(self, msg: Image):
        """Callback para imagem de profundidade do Spot Driver."""
        self.latest_depth_image = msg
        
    def camera_info_callback(self, msg: CameraInfo):
        """Callback para informações da câmera do Spot Driver."""
        self.latest_camera_info = msg
        
    def manipulation_state_callback(self, msg: ManipulationState):
        """Callback para estado de manipulação do Spot Driver."""
        # Aqui você pode monitorar o estado do gripper e braço
        pass
        
    def hand_orientation_callback(self, msg: PoseStamped):
        """Callback para orientação da mão do hand_orientation_estimator."""
        self.current_hand_orientation = msg.pose.orientation
            
    def gesture_callback(self, msg: GestureCommand):
        """Callback para comandos de gesto."""
        self.current_gesture = msg.gesture_type
        
        # Atualiza modo baseado no gesto
        if msg.gesture_type == 2:  # modo_manual
            self.control_mode = ControlMode.MANUAL
        elif msg.gesture_type == 3:  # modo_autonomo
            self.control_mode = ControlMode.AUTONOMOUS
        elif msg.gesture_type == 1:  # fechar gripper
            self.close_gripper()
        elif msg.gesture_type == 0:  # abrir gripper
            self.open_gripper()
            
    def object_detection_callback(self, msg: ObjectDetection):
        """Callback para detecções de objetos (campos de potencial)."""
        if msg.pose_3d:
            # Converte para coordenadas odom
            try:
                transform = self.tf_buffer.lookup_transform(
                    'odom', msg.pose_3d.header.frame_id,
                    rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0)
                )
                
                pose_odom = tf2_geometry_msgs.do_transform_pose(msg.pose_3d, transform)
                
                self.pf_target_odom = np.array([
                    pose_odom.pose.position.x,
                    pose_odom.pose.position.y,
                    pose_odom.pose.position.z
                ])
                self.last_detection_time = time.time()
                
            except Exception as e:
                self.get_logger().warn(f"Falha ao transformar pose do objeto: {e}")
                
    def grasp_result_callback(self, msg: GraspResult):
        """Callback para resultados de grasp."""
        if msg.success:
            self.enter_manipulation_mode()
        else:
            self.get_logger().warn(f"Grasp falhou: {msg.error_message}")
            
    def planning_scene_callback(self, msg: moveit_msgs.msg.PlanningScene):
        """Callback para detecção de colisão."""
        # O campo world.collision_objects fica vazio quando NÃO há colisão
        self.is_colliding = len(msg.world.collision_objects) > 0
        
    def control_loop(self):
        """Loop principal de controle."""
        now = time.time()
        
        # Verifica se estamos no modo de manipulação
        if self.manipulation_mode:
            self.manipulation_control_step()
            return
            
        # Verifica gestos para iniciar grasp
        if self.current_gesture == 1:
            self.get_logger().info("✋ Gesto de agarrar detectado!")
            
            # Congela orientação atual antes de fechar a garra
            sim_pose = self.get_current_pose()
            if sim_pose:
                self.frozen_orientation = sim_pose.orientation
                self.is_orientation_locked = True
                self.get_logger().info("🔒 Orientação congelada durante fechamento da garra")

            # Fecha a garra e espera resposta
            self.close_gripper()

            self.get_logger().info("✅ Garra fechada. Voltando a liberar orientação")
            self.is_orientation_locked = False

            # Executa grasp (aqui você integraria com o GraspManager)
            # Por enquanto, simula sucesso
            self.enter_manipulation_mode()
            
            # Reseta o gesto
            self.current_gesture = 0
            return
            
        # Modo manual com campos de potencial
        if self.control_mode == ControlMode.MANUAL:
            self.manual_control_step()
        elif self.control_mode == ControlMode.AUTONOMOUS:
            self.autonomous_control_step()
            
    def manual_control_step(self):
        """Passo de controle manual com campos de potencial."""
        # Atualiza alvo de campos de potencial a cada 1 segundo
        now = time.time()
        if now - self.last_detection_time > 1.0:
            new_target = self._yolo_depth_deproject()
            if new_target is not None:
                self.pf_target_odom = new_target
                self.last_detection_time = now
            elif now - self.last_detection_time > self.target_valid_duration:
                self.pf_target_odom = None  # Limpa alvo se muito antigo

        # Comando de gripper baseado no gesto
        if self.current_gesture == 0:
            self.open_gripper()
        else:
            self.close_gripper()

        # Obtém pose atual
        current_pose = self.get_current_pose()
        if current_pose is None:
            return
            
        current_pos = np.array([
            current_pose.position.x,
            current_pose.position.y,
            current_pose.position.z
        ])
        
        # Campos de potencial
        use_pf = (self.pf_target_odom is not None and self.current_gesture == 0)
        
        if use_pf:
            dist = np.linalg.norm(self.pf_target_odom - current_pos)
            # Atrai só se estiver MAIS PERTO que 40 cm
            if dist < self.pf_attraction_distance:
                delta = self.compute_attractive_force(current_pos, self.pf_target_odom)
                new_pos = current_pos + delta
            else:
                new_pos = current_pos
        else:
            new_pos = current_pos
            
        # Aplica orientação
        if self.is_orientation_locked and self.frozen_orientation:
            orientation = self.frozen_orientation
        else:
            orientation = current_pose.orientation
            
        # ENVIA COMANDO VIA SPOT DRIVER ROS2
        self.send_arm_command(new_pos, orientation)
        
        # Mostra debug overlay
        self._show_debug_overlay(use_pf)
        
    def autonomous_control_step(self):
        """Passo de controle autônomo."""
        # Mantém pose atual ou executa escape
        pass
        
    def manipulation_control_step(self):
        """Passo de controle de manipulação."""
        # Verifica se deve sair do modo de manipulação
        if self.current_gesture == 0:
            self.exit_manipulation_mode()
        else:
            # Continua no modo de manipulação
            current_pose = self.get_current_pose()
            if current_pose:
                self.send_arm_command(
                    np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z]),
                    current_pose.orientation
                )
                
    def compute_attractive_force(self, current: np.ndarray, target: np.ndarray) -> np.ndarray:
        """Compute a gentle attractive force."""
        return self.pf_k_att * (target - current)
        
    def get_current_pose(self) -> Optional[Pose]:
        """Obtém a pose atual do pulso humano via TF do arm_pose_estimator."""
        try:
            # Obtém pose do pulso humano via TF (arm_pose_estimator)
            transform = self.tf_buffer.lookup_transform(
                'body', 'wrist', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0)
            )
            
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            
            # Usa orientação da mão se disponível, senão usa a do TF
            if self.current_hand_orientation is not None:
                pose.orientation = self.current_hand_orientation
            else:
                pose.orientation = transform.transform.rotation
            
            return pose
            
        except Exception as e:
            self.get_logger().warn(f"Falha ao obter pose do pulso via TF: {e}")
            return None
            
    def _yolo_depth_deproject(self):
        """Detecta o objeto em 'hand_color_image' e retorna as coordenadas 3D no quadro de coordenadas do robô."""
        if (self.latest_rgb_image is None or 
            self.latest_depth_image is None or 
            self.latest_camera_info is None or
            self.model is None):
            return None
            
        try:
            # Converte imagens
            rgb_cv = self.bridge.imgmsg_to_cv2(self.latest_rgb_image, desired_encoding='bgr8')
            depth_cv = self.bridge.imgmsg_to_cv2(self.latest_depth_image, desired_encoding='16UC1')
            
            # Detecta objetos na imagem RGB
            results = self.model(cv2.cvtColor(rgb_cv, cv2.COLOR_BGR2RGB))
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
            depth_rows, depth_cols = depth_cv.shape
            cx_safe = min(max(0, int(cx)), depth_cols - 1)
            cy_safe = min(max(0, int(cy)), depth_rows - 1)
            
            raw_mm = int(depth_cv[cy_safe, cx_safe])
            
            # Filtra profundidade
            min_depth_mm = 300   # 30 cm
            max_depth_mm = 3000  # 3 m
            
            depth_mm = raw_mm if (min_depth_mm <= raw_mm <= max_depth_mm) else min_depth_mm
            depth_m = depth_mm / 1000.0
            
            # De-projeção do pixel -> quadro de câmera usando camera_info
            fx = self.latest_camera_info.k[0]  # focal length x
            fy = self.latest_camera_info.k[4]  # focal length y
            cx0 = self.latest_camera_info.k[2]  # principal point x
            cy0 = self.latest_camera_info.k[5]  # principal point y
            
            x_cam = (cx - cx0) * depth_m / fx
            y_cam = (cy - cy0) * depth_m / fy
            z_cam = depth_m
            
            # Quadro de câmera → quadro de odom
            try:
                transform = self.tf_buffer.lookup_transform(
                    'odom', self.latest_camera_info.header.frame_id,
                    rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0)
                )
                
                # Cria pose do objeto no quadro da câmera
                pose_cam = PoseStamped()
                pose_cam.header.frame_id = self.latest_camera_info.header.frame_id
                pose_cam.header.stamp = self.get_clock().now().to_msg()
                pose_cam.pose.position.x = x_cam
                pose_cam.pose.position.y = y_cam
                pose_cam.pose.position.z = z_cam
                pose_cam.pose.orientation.w = 1.0
                
                # Transforma para odom
                pose_odom = tf2_geometry_msgs.do_transform_pose(pose_cam, transform)
                
                return np.array([pose_odom.pose.position.x, pose_odom.pose.position.y, pose_odom.pose.position.z])
                
            except Exception as e:
                self.get_logger().warn(f"Falha ao transformar pose: {e}")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Erro na de-projeção YOLO: {e}")
            return None
            
    def _filter_allowed_objects(self, boxes, names):
        """Filtra objetos detectados mantendo apenas os permitidos."""
        if self.allowed_objects is None:
            return []
            
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
        
    def _show_debug_overlay(self, pf_active: bool):
        """Displays image with bounding box, distance, and PF status."""
        if self.latest_rgb_image is None:
            return  # Nothing to display

        try:
            # Cria visualização para imagem original
            vis_orig = self.bridge.imgmsg_to_cv2(self.latest_rgb_image, desired_encoding='bgr8').copy()
            
            # Adiciona indicação da orientação da garra (como uma seta)
            sim_pose = self.get_current_pose()
            if sim_pose is not None:
                h, w = vis_orig.shape[:2]
                center_x, center_y = w // 2, h // 2
                arrow_length = 50
                quat = sim_pose.orientation
                # Cria vetor de direção simplificado a partir do quaternion
                dx = 2 * (quat.x * quat.z + quat.w * quat.y)
                dy = 2 * (quat.y * quat.z - quat.w * quat.x)
                
                # Desenha seta de orientação
                end_x = int(center_x + arrow_length * dx)
                end_y = int(center_y + arrow_length * dy)
                cv2.arrowedLine(vis_orig, (center_x, center_y), (end_x, end_y), (255, 0, 0), 2)

            # Display PF status na imagem original
            txt = "PF: ON" if pf_active else "PF: OFF"
            cv2.putText(vis_orig, txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (255, 255, 0), 2)
            cv2.putText(vis_orig, "ORIGINAL", (10, 60), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (255, 255, 0), 2)
                
            cv2.imshow("debug_view", vis_orig)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().warn(f"Erro ao mostrar debug overlay: {e}")
            
    def send_arm_command(self, position: np.ndarray, orientation):
        """Envia comando de posição para o braço via Spot Driver ROS2."""
        try:
            # Cria mensagem JointState para comandos de braço
            joint_state = JointState()
            joint_state.name = ["arm_sh0", "arm_sh1", "arm_el0", "arm_el1", "arm_wr0", "arm_wr1", "arm_f1x"]
            
            # Calcula posições das juntas usando MoveIt (simulação)
            if self.moveit_group:
                # Cria pose alvo
                target_pose = Pose()
                target_pose.position.x = position[0]
                target_pose.position.y = position[1]
                target_pose.position.z = position[2]
                target_pose.orientation = orientation
                
                # Planeja trajetória
                self.moveit_group.set_pose_target(target_pose)
                success, plan, _ = self.moveit_group.plan()
                
                if success and plan.joint_trajectory.points:
                    # Pega as posições das juntas do plano
                    joint_positions = plan.joint_trajectory.points[-1].positions
                    joint_state.position = joint_positions
                    
                    # Publica comando via Spot Driver
                    self.arm_joint_pub.publish(joint_state)
                    
                    self.get_logger().debug(
                        f"Comando de braço enviado: pos=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f})"
                    )
                else:
                    self.get_logger().warn("Falha ao planejar trajetória para o braço")
                    
        except Exception as e:
            self.get_logger().error(f"Falha ao enviar comando de braço: {e}")
        
    def open_gripper(self):
        """Abre a garra via Spot Driver ROS2."""
        self.get_logger().info("Abrindo garra...")
        request = Trigger.Request()
        future = self.open_gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.done():
            response = future.result()
            if response.success:
                self.get_logger().info("✅ Garra aberta com sucesso")
            else:
                self.get_logger().warn("⚠️ Falha ao abrir garra")
        
    def close_gripper(self, block: bool = False, timeout_sec: float = 2.0):
        """Fecha a garra via Spot Driver ROS2."""
        self.get_logger().info("Fechando garra...")
        request = Trigger.Request()
        future = self.close_gripper_client.call_async(request)
        
        if block:
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            if future.done():
                response = future.result()
                if response.success:
                    self.get_logger().info("✅ Garra fechada com sucesso")
                else:
                    self.get_logger().warn("⚠️ Falha ao fechar garra")
        
    def enter_manipulation_mode(self):
        """Entra no modo de manipulação."""
        self.manipulation_mode = True
        self.control_mode = ControlMode.MANIPULATION
        self.get_logger().info("🤖 Entrando no modo de MANIPULAÇÃO.")
        
    def exit_manipulation_mode(self):
        """Sai do modo de manipulação."""
        # Abre a garra por 3 segundos
        self.get_logger().info("Liberando objeto: Abrindo garra por 3 segundos...")
        self.open_gripper()
        time.sleep(3.0)
        
        # Fecha a garra novamente com bloqueio
        self.get_logger().info("Fechando garra...")
        self.close_gripper(block=True)
        
        # Sai do modo de manipulação
        self.manipulation_mode = False
        self.control_mode = ControlMode.MANUAL
        self.get_logger().info("🤖 Saindo do modo de MANIPULAÇÃO. Retornando ao modo normal.")
        
    def escape_via_moveit(self):
        """Planeja 20 cm para cima (eixo –Z do EE) e executa."""
        # Obtém pose atual via TF do arm_pose_estimator
        current_pose = self.get_current_pose()
        if current_pose is None:
            self.get_logger().warn("Não foi possível obter pose atual para escape")
            return
            
        # Cria pose alvo 20cm acima
        target_pose = copy.deepcopy(current_pose)
        target_pose.position.z += self.escape_distance

        # Envia comando de escape via Spot Driver
        self.send_arm_command(
            np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z]),
            target_pose.orientation
        )
        
        self.get_logger().info("Comando de escape enviado")

    def get_control_state_callback(self, request, response):
        """Callback para o serviço de obtenção do estado de controle."""
        response.current_mode = self.control_mode.value
        response.is_manipulation_mode = self.manipulation_mode
        response.current_gesture = self.current_gesture
        response.is_orientation_locked = self.is_orientation_locked
        response.pf_target_active = self.pf_target_odom is not None
        response.is_colliding = self.is_colliding
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SpotControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 