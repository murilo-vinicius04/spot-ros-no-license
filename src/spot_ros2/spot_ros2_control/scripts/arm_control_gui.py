#!/usr/bin/env python3
"""
Simple GUI for controlling Spot's arm using ros2_control.
This script provides sliders for each arm joint and buttons for preset positions.
"""

import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
    GUI_AVAILABLE = True
except ImportError:
    GUI_AVAILABLE = False
    print("Warning: tkinter not available. GUI will not work.")


class SpotArmController(Node):
    """ROS2 node for controlling Spot's arm"""
    
    def __init__(self):
        super().__init__('spot_arm_controller')
        
        # Arm joint names in order
        self.arm_joints = [
            'arm_sh0',  # Shoulder 0 (yaw)
            'arm_sh1',  # Shoulder 1 (pitch)
            'arm_el0',  # Elbow 0 (pitch)
            'arm_el1',  # Elbow 1 (roll)
            'arm_wr0',  # Wrist 0 (pitch)
            'arm_wr1',  # Wrist 1 (roll)
            'arm_f1x',  # Finger (gripper)
        ]
        
        # Joint limits (radians)
        self.joint_limits = {
            'arm_sh0': (-2.618, 3.142),
            'arm_sh1': (-3.142, 0.524),
            'arm_el0': (0.0, 3.142),
            'arm_el1': (-2.793, 2.793),
            'arm_wr0': (-1.833, 1.833),
            'arm_wr1': (-2.880, 2.880),
            'arm_f1x': (-1.570, 0.0),
        }
        
        # Current joint positions
        self.current_positions = [0.0] * len(self.arm_joints)
        
        # Publishers and subscribers
        self.command_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info("Spot Arm Controller initialized")
    
    def joint_state_callback(self, msg):
        """Update current joint positions from joint state feedback"""
        try:
            for i, joint_name in enumerate(self.arm_joints):
                if joint_name in msg.name:
                    idx = msg.name.index(joint_name)
                    if idx < len(msg.position):
                        self.current_positions[i] = msg.position[idx]
        except Exception as e:
            self.get_logger().warn(f"Error parsing joint states: {e}")
    
    def send_joint_command(self, positions):
        """Send joint position commands to the robot"""
        if len(positions) != len(self.arm_joints):
            self.get_logger().error(f"Expected {len(self.arm_joints)} positions, got {len(positions)}")
            return
        
        # Clamp positions to joint limits
        clamped_positions = []
        for i, pos in enumerate(positions):
            joint_name = self.arm_joints[i]
            min_pos, max_pos = self.joint_limits[joint_name]
            clamped_pos = max(min_pos, min(max_pos, pos))
            clamped_positions.append(clamped_pos)
        
        # Create and publish command
        cmd_msg = Float64MultiArray()
        cmd_msg.data = clamped_positions
        self.command_pub.publish(cmd_msg)
        
        self.get_logger().info(f"Sent joint command: {[f'{p:.3f}' for p in clamped_positions]}")
    
    def get_preset_positions(self, preset_name):
        """Get predefined joint positions for common poses"""
        presets = {
            'stow': [0.0, -2.0, 2.5, 0.0, 1.5, 0.0, -0.5],
            'carry': [0.0, -1.2, 1.8, 0.0, 0.8, 0.0, -0.3],
            'ready': [0.0, -0.5, 1.0, 0.0, 0.5, 0.0, -0.2],
            'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        }
        return presets.get(preset_name, [0.0] * len(self.arm_joints))


class SpotArmGUI:
    """GUI for controlling Spot's arm"""
    
    def __init__(self, controller):
        self.controller = controller
        self.root = tk.Tk()
        self.root.title("Spot Arm Control")
        self.root.geometry("800x600")
        
        # Auto-send setting
        self.auto_send = tk.BooleanVar(value=False)  # Default to manual mode
        self.last_send_time = time.time()
        self.send_delay = 0.1  # Minimum delay between auto-sends (100ms)
        
        # Create GUI elements
        self.create_widgets()
        
        # Update timer
        self.update_display()
    
    def create_widgets(self):
        """Create all GUI widgets"""
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title_label = ttk.Label(main_frame, text="Spot Arm Controller", font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=3, pady=(0, 20))
        
        # Auto-send option
        auto_frame = ttk.Frame(main_frame)
        auto_frame.grid(row=1, column=0, columnspan=3, pady=(0, 10))
        
        auto_check = ttk.Checkbutton(
            auto_frame,
            text="🔄 Auto-send commands (real-time slider control)",
            variable=self.auto_send
        )
        auto_check.grid(row=0, column=0)
        
        warning_label = ttk.Label(
            auto_frame,
            text="⚠️ Auto-send: be careful with movements!",
            foreground="orange",
            font=("Arial", 8)
        )
        warning_label.grid(row=1, column=0)
        
        # Joint control section
        joint_frame = ttk.LabelFrame(main_frame, text="Joint Control", padding="10")
        joint_frame.grid(row=2, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        self.joint_vars = []
        self.joint_scales = []
        
        for i, joint_name in enumerate(self.controller.arm_joints):
            min_pos, max_pos = self.controller.joint_limits[joint_name]
            
            # Joint label
            label = ttk.Label(joint_frame, text=f"{joint_name}:")
            label.grid(row=i, column=0, sticky=tk.W, padx=(0, 10))
            
            # Joint scale
            var = tk.DoubleVar()
            scale = tk.Scale(
                joint_frame,
                from_=min_pos,
                to=max_pos,
                resolution=0.01,
                orient=tk.HORIZONTAL,
                variable=var,
                length=300,
                command=self.on_joint_change
            )
            scale.grid(row=i, column=1, padx=(0, 10))
            
            # Current position label
            pos_label = ttk.Label(joint_frame, text="0.000")
            pos_label.grid(row=i, column=2, sticky=tk.W)
            
            self.joint_vars.append(var)
            self.joint_scales.append((scale, pos_label))
        
        # Preset buttons
        preset_frame = ttk.LabelFrame(main_frame, text="Preset Positions", padding="10")
        preset_frame.grid(row=3, column=0, columnspan=3, sticky=(tk.W, tk.E), pady=(0, 10))
        
        presets = ['stow', 'carry', 'ready', 'home']
        for i, preset in enumerate(presets):
            btn = ttk.Button(
                preset_frame,
                text=preset.title(),
                command=lambda p=preset: self.apply_preset(p)
            )
            btn.grid(row=0, column=i, padx=5)
        
        # Control buttons
        control_frame = ttk.Frame(main_frame)
        control_frame.grid(row=4, column=0, columnspan=3, pady=(0, 10))
        
        send_btn = ttk.Button(
            control_frame,
            text="📤 Send Command",
            command=self.send_command
        )
        send_btn.grid(row=0, column=0, padx=(0, 10))
        
        stop_btn = ttk.Button(
            control_frame,
            text="🛑 Stop",
            command=self.stop_robot
        )
        stop_btn.grid(row=0, column=1, padx=(0, 10))
        
        current_btn = ttk.Button(
            control_frame,
            text="📍 Set to Current",
            command=self.set_to_current
        )
        current_btn.grid(row=0, column=2, padx=(0, 10))
        
        # Status
        self.status_label = ttk.Label(main_frame, text="Status: Ready", foreground="green")
        self.status_label.grid(row=5, column=0, columnspan=3)
    
    def on_joint_change(self, value):
        """Called when a joint slider changes"""
        if self.auto_send.get():
            current_time = time.time()
            if current_time - self.last_send_time >= self.send_delay:
                self.send_command()
                self.last_send_time = current_time
    
    def set_to_current(self):
        """Set all sliders to current joint positions"""
        for i, var in enumerate(self.joint_vars):
            current_pos = self.controller.current_positions[i]
            var.set(current_pos)
        
        self.status_label.config(text="Sliders set to current positions", foreground="blue")
    
    def apply_preset(self, preset_name):
        """Apply a preset position"""
        positions = self.controller.get_preset_positions(preset_name)
        
        # Update sliders
        for i, pos in enumerate(positions):
            self.joint_vars[i].set(pos)
        
        # Send command
        self.send_command()
        self.status_label.config(text=f"Applied preset: {preset_name}", foreground="blue")
    
    def send_command(self):
        """Send current slider positions to robot"""
        positions = [var.get() for var in self.joint_vars]
        self.controller.send_joint_command(positions)
        self.status_label.config(text="Command sent", foreground="green")
    
    def stop_robot(self):
        """Send stop command (current positions)"""
        self.controller.send_joint_command(self.controller.current_positions)
        self.status_label.config(text="Stop command sent", foreground="orange")
    
    def update_display(self):
        """Update the display with current joint positions"""
        for i, (scale, pos_label) in enumerate(self.joint_scales):
            current_pos = self.controller.current_positions[i]
            pos_label.config(text=f"{current_pos:.3f}")
        
        # Schedule next update
        self.root.after(100, self.update_display)
    
    def run(self):
        """Start the GUI"""
        self.root.mainloop()


def main():
    """Main function"""
    if not GUI_AVAILABLE:
        print("ERROR: GUI not available. Please install tkinter.")
        return 1
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create controller node
        controller = SpotArmController()
        
        # Start ROS2 spinning in a separate thread
        executor = MultiThreadedExecutor()
        executor.add_node(controller)
        
        ros_thread = threading.Thread(target=executor.spin, daemon=True)
        ros_thread.start()
        
        # Give ROS some time to initialize
        time.sleep(1.0)
        
        # Create and run GUI
        gui = SpotArmGUI(controller)
        print("Starting Spot Arm Control GUI...")
        print("Available commands:")
        print("- Use sliders to control individual joints")
        print("- Click preset buttons for common positions")
        print("- Click 'Send Command' to execute")
        print("- Click 'Stop' to halt movement")
        
        gui.run()
        
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Error: {e}")
        return 1
    finally:
        # Cleanup
        if 'controller' in locals():
            controller.destroy_node()
        rclpy.shutdown()
    
    return 0


if __name__ == '__main__':
    sys.exit(main()) 