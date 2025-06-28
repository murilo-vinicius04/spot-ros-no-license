#!/usr/bin/env python3

# Copyright (c) 2024 Boston Dynamics AI Institute LLC. All rights reserved.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time
import math

class CustomPositionController(Node):
    """
    Nó que publica uma posição personalizada específica para o robô Spot
    usando ros2_control através do forward_position_controller
    """
    
    def __init__(self):
        super().__init__('custom_position_controller')
        
        # Parâmetros configuráveis
        self.spot_name = self.declare_parameter('spot_name', '').get_parameter_value().string_value
        self.command_rate = self.declare_parameter('command_rate', 50.0).get_parameter_value().double_value
        
        # Posição personalizada desejada (ângulos das juntas em radianos)
        # Ordem: FL_hip_x, FL_hip_y, FL_knee, FR_hip_x, FR_hip_y, FR_knee, 
        #        RL_hip_x, RL_hip_y, RL_knee, RR_hip_x, RR_hip_y, RR_knee
        self.custom_position = self.declare_parameter(
            'custom_position',
            [0.0, 0.5, -1.0, 0.0, 0.5, -1.0, 0.0, 0.5, -1.0, 0.0, 0.5, -1.0]
        ).get_parameter_value().double_array_value
        
        # Duração para alcançar a posição (segundos)
        self.transition_duration = self.declare_parameter('transition_duration', 3.0).get_parameter_value().double_value
        
        # Estado interno
        self.njoints = 12
        self.initialized = False
        self.initial_position = None
        self.start_time = None
        self.target_reached = False
        
        # Validação dos parâmetros
        if len(self.custom_position) != self.njoints:
            raise ValueError(f"custom_position deve ter exatamente {self.njoints} valores")
        
        # Publishers e Subscribers
        topic_name = 'forward_position_controller/commands'
        if self.spot_name:
            topic_name = f'{self.spot_name}/{topic_name}'
            
        self.command_publisher = self.create_publisher(
            Float64MultiArray, 
            topic_name, 
            10
        )
        
        joint_states_topic = 'low_level/joint_states'
        if self.spot_name:
            joint_states_topic = f'{self.spot_name}/{joint_states_topic}'
            
        self.joint_states_subscriber = self.create_subscription(
            JointState,
            joint_states_topic,
            self.joint_states_callback,
            10
        )
        
        # Timer para publicar comandos
        timer_period = 1.0 / self.command_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.get_logger().info(f"Custom Position Controller iniciado")
        self.get_logger().info(f"Posição alvo: {self.custom_position}")
        self.get_logger().info(f"Aguardando joint states para inicializar...")

    def joint_states_callback(self, msg):
        """Callback para receber os estados das juntas e inicializar"""
        if not self.initialized:
            # Mapear os joint states para a ordem correta
            ordered_position = self.order_joint_states(msg)
            if ordered_position is not None:
                self.initial_position = ordered_position
                self.start_time = self.get_clock().now()
                self.initialized = True
                self.get_logger().info("Inicializado! Movendo para posição personalizada...")
                self.get_logger().info(f"Posição inicial: {self.initial_position}")

    def order_joint_states(self, msg):
        """
        Reordena os joint states para a ordem esperada pelo controlador
        Baseado na função spot_ros2_control::order_joint_states
        """
        # Ordem esperada das juntas
        expected_joints = [
            'front_left_hip_x', 'front_left_hip_y', 'front_left_knee',
            'front_right_hip_x', 'front_right_hip_y', 'front_right_knee',
            'rear_left_hip_x', 'rear_left_hip_y', 'rear_left_knee',
            'rear_right_hip_x', 'rear_right_hip_y', 'rear_right_knee'
        ]
        
        if self.spot_name:
            # Adicionar prefixo se necessário
            expected_joints = [f'{self.spot_name}/{joint}' for joint in expected_joints]
        
        ordered_position = []
        
        for joint_name in expected_joints:
            try:
                index = msg.name.index(joint_name)
                ordered_position.append(msg.position[index])
            except ValueError:
                self.get_logger().warn(f"Junta {joint_name} não encontrada nos joint states")
                return None
        
        return ordered_position

    def interpolate_position(self, start_pos, end_pos, progress):
        """Interpola linearmente entre duas posições"""
        return [start + progress * (end - start) for start, end in zip(start_pos, end_pos)]

    def timer_callback(self):
        """Callback do timer para publicar comandos"""
        if not self.initialized:
            return
        
        if self.target_reached:
            # Continua publicando a posição alvo para manter o robô na posição
            command = Float64MultiArray()
            command.data = self.custom_position
            self.command_publisher.publish(command)
            return
        
        # Calcular progresso da transição (0.0 a 1.0)
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9
        progress = min(elapsed_time / self.transition_duration, 1.0)
        
        # Interpolar entre posição inicial e posição alvo
        current_position = self.interpolate_position(
            self.initial_position,
            self.custom_position,
            progress
        )
        
        # Publicar comando
        command = Float64MultiArray()
        command.data = current_position
        self.command_publisher.publish(command)
        
        # Verificar se chegou ao destino
        if progress >= 1.0 and not self.target_reached:
            self.target_reached = True
            self.get_logger().info("Posição personalizada alcançada!")
            self.get_logger().info(f"Posição final: {self.custom_position}")

def main(args=None):
    rclpy.init(args=args)
    
    try:
        controller = CustomPositionController()
        rclpy.spin(controller)
    except Exception as e:
        print(f"Erro: {e}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main() 