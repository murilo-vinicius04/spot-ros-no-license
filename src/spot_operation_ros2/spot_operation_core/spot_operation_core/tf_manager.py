#!/usr/bin/env python3
"""
Gerenciador de transformações de coordenadas usando TF.
Baseado no código original do TFManager.
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped, Pose
from std_msgs.msg import Header
import numpy as np
from typing import Optional, Tuple


class TFManager(Node):
    """Gerencia transformações de coordenadas usando TF."""
    
    def __init__(self):
        super().__init__('tf_manager')
        
        # Parâmetros
        self.declare_parameter('default_timeout', 1.0)
        self.declare_parameter('default_reference_frame', 'body')
        self.declare_parameter('default_target_frame', 'wrist')
        
        self.default_timeout = self.get_parameter('default_timeout').value
        self.default_reference_frame = self.get_parameter('default_reference_frame').value
        self.default_target_frame = self.get_parameter('default_target_frame').value
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Cache para transformações frequentes
        self._transform_cache = {}
        self._cache_timeout = 0.1  # 100ms
        
        self.get_logger().info("TFManager inicializado")
        
    def get_pose(self, target_frame: str = None, reference_frame: str = None, 
                 timeout: float = None) -> Optional[Pose]:
        """
        Obtém a pose de um frame em relação a outro usando TF.
        
        Args:
            target_frame: Frame de destino (padrão: 'wrist')
            reference_frame: Frame de referência (padrão: 'body')
            timeout: Timeout para lookup (padrão: 1.0s)
            
        Returns:
            Pose do frame de destino em relação ao frame de referência, ou None se falhar
        """
        if target_frame is None:
            target_frame = self.default_target_frame
        if reference_frame is None:
            reference_frame = self.default_reference_frame
        if timeout is None:
            timeout = self.default_timeout
            
        try:
            # Verifica cache primeiro
            cache_key = f"{target_frame}_{reference_frame}"
            if cache_key in self._transform_cache:
                cached_transform, timestamp = self._transform_cache[cache_key]
                if (self.get_clock().now().nanoseconds - timestamp) < (self._cache_timeout * 1e9):
                    return self._transform_to_pose(cached_transform)
            
            # Lookup da transformação
            transform = self.tf_buffer.lookup_transform(
                reference_frame, target_frame, rclpy.time.Time(), 
                rclpy.duration.Duration(seconds=timeout)
            )
            
            # Atualiza cache
            self._transform_cache[cache_key] = (transform, self.get_clock().now().nanoseconds)
            
            return self._transform_to_pose(transform)
            
        except Exception as e:
            self.get_logger().warn(f"TF lookup falhou: {e}")
            return None
    
    def get_transform(self, target_frame: str, reference_frame: str, 
                     timeout: float = None) -> Optional[TransformStamped]:
        """
        Obtém a transformação entre dois frames.
        
        Args:
            target_frame: Frame de destino
            reference_frame: Frame de referência
            timeout: Timeout para lookup
            
        Returns:
            TransformStamped ou None se falhar
        """
        if timeout is None:
            timeout = self.default_timeout
            
        try:
            transform = self.tf_buffer.lookup_transform(
                reference_frame, target_frame, rclpy.time.Time(),
                rclpy.duration.Duration(seconds=timeout)
            )
            return transform
        except Exception as e:
            self.get_logger().warn(f"TF lookup falhou: {e}")
            return None
    
    def transform_pose(self, pose: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        """
        Transforma uma pose para um frame de destino.
        
        Args:
            pose: Pose a ser transformada
            target_frame: Frame de destino
            
        Returns:
            PoseStamped transformada ou None se falhar
        """
        try:
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose, 
                self.tf_buffer.lookup_transform(target_frame, pose.header.frame_id, 
                                              rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0)))
            return transformed_pose
        except Exception as e:
            self.get_logger().warn(f"Transformação de pose falhou: {e}")
            return None
    
    def broadcast_transform(self, transform: TransformStamped):
        """
        Publica uma transformação.
        
        Args:
            transform: TransformStamped a ser publicada
        """
        self.tf_broadcaster.sendTransform(transform)
    
    def broadcast_pose_as_transform(self, pose: PoseStamped, child_frame: str, 
                                  parent_frame: str = None):
        """
        Publica uma pose como transformação.
        
        Args:
            pose: Pose a ser publicada como transformação
            child_frame: Frame filho
            parent_frame: Frame pai (padrão: frame da pose)
        """
        if parent_frame is None:
            parent_frame = pose.header.frame_id
            
        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = child_frame
        transform.header.frame_id = parent_frame
        
        transform.transform.translation.x = pose.pose.position.x
        transform.transform.translation.y = pose.pose.position.y
        transform.transform.translation.z = pose.pose.position.z
        
        transform.transform.rotation = pose.pose.orientation
        
        self.broadcast_transform(transform)
    
    def _transform_to_pose(self, transform: TransformStamped) -> Pose:
        """Converte TransformStamped para Pose."""
        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose
    
    def get_pose_stamped(self, target_frame: str = None, reference_frame: str = None,
                        timeout: float = None) -> Optional[PoseStamped]:
        """
        Obtém a pose stamped de um frame em relação a outro.
        
        Args:
            target_frame: Frame de destino
            reference_frame: Frame de referência
            timeout: Timeout para lookup
            
        Returns:
            PoseStamped ou None se falhar
        """
        if target_frame is None:
            target_frame = self.default_target_frame
        if reference_frame is None:
            reference_frame = self.default_reference_frame
        if timeout is None:
            timeout = self.default_timeout
            
        try:
            transform = self.tf_buffer.lookup_transform(
                reference_frame, target_frame, rclpy.time.Time(),
                rclpy.duration.Duration(seconds=timeout)
            )
            
            pose_stamped = PoseStamped()
            pose_stamped.header = transform.header
            pose_stamped.pose = self._transform_to_pose(transform)
            
            return pose_stamped
            
        except Exception as e:
            self.get_logger().warn(f"TF lookup falhou: {e}")
            return None
    
    def wait_for_transform(self, target_frame: str, reference_frame: str, 
                          timeout: float = 10.0) -> bool:
        """
        Espera por uma transformação estar disponível.
        
        Args:
            target_frame: Frame de destino
            reference_frame: Frame de referência
            timeout: Timeout em segundos
            
        Returns:
            True se a transformação estiver disponível, False se timeout
        """
        try:
            self.tf_buffer.can_transform(reference_frame, target_frame, 
                                       rclpy.time.Time(), rclpy.duration.Duration(seconds=timeout))
            return True
        except Exception:
            return False
    
    def clear_cache(self):
        """Limpa o cache de transformações."""
        self._transform_cache.clear()
        self.get_logger().debug("Cache de TF limpo")


def main(args=None):
    rclpy.init(args=args)
    tf_manager = TFManager()
    
    try:
        rclpy.spin(tf_manager)
    except KeyboardInterrupt:
        pass
    finally:
        tf_manager.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 