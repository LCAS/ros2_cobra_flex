#!/usr/bin/env python3
"""
Cobra Flex ROS2 Driver Node

A ROS2 node for the Waveshare Cobra Flex chassis that:
- Communicates with ESP32 via JSON over serial
- Subscribes to /cmd_vel for velocity commands
- Publishes /odom for odometry
- Publishes /imu for IMU data
- Publishes /joint_states for wheel states

Uses Waveshare JSON protocol:
- T=1: Movement command (L, R wheel speeds in 0.1rpm units)
- T=11: Individual motor control (M1-M4 in 0.1rpm units)
- T=130: Request feedback
- T=131: Continuous feedback control
- T=132: LED brightness control
- T=1001: Feedback response (M1-M4, odl, odr, v)

Based on Waveshare UGV architecture and JSON protocol.
Reference: https://www.waveshare.com/wiki/Cobra_Flex#JSON_Communication_Protocol

Author: L-CAS, University of Lincoln
License: GPL-3.0
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import serial
import json
import threading
import math
import time
from dataclasses import dataclass
from typing import Optional, Dict, Any

# ROS2 message types
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster


@dataclass
class RobotState:
    """Stores current robot state from ESP32"""
    linear_velocity: float = 0.0
    angular_velocity: float = 0.0
    position_x: float = 0.0
    position_y: float = 0.0
    orientation_z: float = 0.0
    
    # Wheel encoder data (4 wheels: FL, FR, RL, RR)
    wheel_positions: list = None
    wheel_velocities: list = None
    
    # IMU data
    imu_orientation: list = None  # [x, y, z, w] quaternion
    imu_angular_velocity: list = None  # [x, y, z] rad/s
    imu_linear_acceleration: list = None  # [x, y, z] m/s²
    
    # Battery and system
    battery_voltage: float = 0.0
    
    def __post_init__(self):
        if self.wheel_positions is None:
            self.wheel_positions = [0.0, 0.0, 0.0, 0.0]
        if self.wheel_velocities is None:
            self.wheel_velocities = [0.0, 0.0, 0.0, 0.0]
        if self.imu_orientation is None:
            self.imu_orientation = [0.0, 0.0, 0.0, 1.0]
        if self.imu_angular_velocity is None:
            self.imu_angular_velocity = [0.0, 0.0, 0.0]
        if self.imu_linear_acceleration is None:
            self.imu_linear_acceleration = [0.0, 0.0, 0.0]


class CobraFlexDriver(Node):
    """Main driver node for Cobra Flex chassis"""
    
    # Robot configuration (based on Waveshare JSON protocol)
    ROBOT_TYPE_COBRA_FLEX = 4  # Adjust based on actual firmware
    MODULE_NONE = 0
    
    # Mechanical parameters (adjust for actual Cobra Flex)
    WHEEL_BASE = 0.225  # metres
    WHEEL_RADIUS = 0.065  # metres
    WHEEL_SEPARATION = 0.225  # metres
    
    def __init__(self):
        super().__init__('cobra_flex_driver')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('cmd_vel_timeout', 1.0)
        self.declare_parameter('publish_rate', 20.0)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        
        # State
        self.robot_state = RobotState()
        self.last_cmd_time = self.get_clock().now()
        self.serial_lock = threading.Lock()
        
        # Serial connection
        self.serial_conn: Optional[serial.Serial] = None
        self.init_serial()
        
        # ROS2 Publishers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', qos)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', qos)
        
        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        
        # Timers
        self.create_timer(0.02, self.read_serial_timer)  # 50 Hz read
        self.create_timer(1.0 / self.publish_rate, self.publish_state_timer)
        self.create_timer(0.5, self.watchdog_timer)
        
        # Initialize robot
        self.send_robot_config()
        
        self.get_logger().info(
            f'Cobra Flex driver initialized on {self.serial_port} @ {self.baud_rate} baud'
        )
    
    def init_serial(self):
        """Initialize serial connection to ESP32"""
        try:
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.1,
                write_timeout=0.1
            )
            time.sleep(2)  # Wait for ESP32 to initialize
            self.get_logger().info('Serial connection established')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            self.serial_conn = None
    
    def send_robot_config(self):
        """Send initial robot configuration to ESP32"""
        # Enable continuous feedback mode
        self.set_continuous_feedback(True)
        time.sleep(0.1)  # Small delay between commands
        
        # Request initial feedback
        self.request_feedback()
    
    def send_json_command(self, command: Dict[str, Any]) -> bool:
        """Send JSON command to ESP32"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            return False
        
        try:
            with self.serial_lock:
                json_str = json.dumps(command) + '\n'
                self.serial_conn.write(json_str.encode('utf-8'))
                self.serial_conn.flush()
            return True
        except Exception as e:
            self.get_logger().error(f'Error sending command: {e}')
            return False
    
    def cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands"""
        self.last_cmd_time = self.get_clock().now()
        
        linear = msg.linear.x  # m/s
        angular = msg.angular.z  # rad/s
        
        # Convert twist to differential drive wheel velocities
        # Using differential drive kinematics: v_left = v - (w * L / 2), v_right = v + (w * L / 2)
        v_left = linear - (angular * self.WHEEL_BASE / 2.0)
        v_right = linear + (angular * self.WHEEL_BASE / 2.0)
        
        # Convert from m/s to rpm: rpm = (v / (2 * pi * r)) * 60
        left_rpm = (v_left / (2 * math.pi * self.WHEEL_RADIUS)) * 60.0
        right_rpm = (v_right / (2 * math.pi * self.WHEEL_RADIUS)) * 60.0
        
        # Convert to 0.1rpm units as per protocol (range: -1800 to 1800 means -180 to 180 rpm)
        left_speed = int(left_rpm * 10)
        right_speed = int(right_rpm * 10)
        
        # Clamp to protocol limits (-1800 to 1800, which is -180 to 180 rpm)
        left_speed = max(-1800, min(1800, left_speed))
        right_speed = max(-1800, min(1800, right_speed))
        
        # Send command with T=1 for speed control, L=left wheel, R=right wheel
        vel_cmd = {
            "T": 1,
            "L": left_speed,
            "R": right_speed
        }
        
        self.send_json_command(vel_cmd)
    
    def send_individual_motor_command(self, m1: int, m2: int, m3: int, m4: int):
        """
        Send individual motor control command (T=11)
        
        Args:
            m1: Left front wheel speed in 0.1rpm units (-1800 to 1800)
            m2: Right front wheel speed in 0.1rpm units (-1800 to 1800)
            m3: Right rear wheel speed in 0.1rpm units (-1800 to 1800)
            m4: Left rear wheel speed in 0.1rpm units (-1800 to 1800)
        """
        motor_cmd = {
            "T": 11,
            "M1": max(-1800, min(1800, m1)),
            "M2": max(-1800, min(1800, m2)),
            "M3": max(-1800, min(1800, m3)),
            "M4": max(-1800, min(1800, m4))
        }
        self.send_json_command(motor_cmd)
    
    def request_feedback(self):
        """Request feedback from robot (T=130)"""
        feedback_cmd = {"T": 130}
        self.send_json_command(feedback_cmd)
    
    def set_continuous_feedback(self, enabled: bool):
        """
        Enable or disable continuous feedback (T=131)
        
        Args:
            enabled: True to enable continuous feedback, False to disable
        """
        continuous_cmd = {
            "T": 131,
            "cmd": 1 if enabled else 0
        }
        self.send_json_command(continuous_cmd)
    
    def set_led_brightness(self, front_led: int, rear_led: int):
        """
        Set LED brightness (T=132)
        
        Args:
            front_led: Front LED brightness (0-255)
            rear_led: Rear LED brightness (0-255)
        """
        led_cmd = {
            "T": 132,
            "IO1": max(0, min(255, front_led)),
            "IO2": max(0, min(255, rear_led))
        }
        self.send_json_command(led_cmd)
    
    def read_serial_timer(self):
        """Read serial data from ESP32"""
        if self.serial_conn is None or not self.serial_conn.is_open:
            return
        
        try:
            with self.serial_lock:
                if self.serial_conn.in_waiting > 0:
                    line = self.serial_conn.readline().decode('utf-8').strip()
                    if line:
                        self.parse_json_response(line)
        except Exception as e:
            self.get_logger().error(f'Error reading serial: {e}')
    
    def parse_json_response(self, json_str: str):
        """Parse JSON response from ESP32"""
        try:
            data = json.loads(json_str)
            
            # Check if this is a feedback response (T=1001)
            if "T" in data and data["T"] == 1001:
                # Parse motor speeds (M1-M4) in 0.1rpm units
                if "M1" in data:
                    # M1 = left front, M2 = right front, M3 = right rear, M4 = left rear
                    # Convert from 0.1rpm to rad/s: rad/s = (rpm / 60) * 2 * pi
                    m1_rpm = data.get("M1", 0) / 10.0  # Convert from 0.1rpm to rpm
                    m2_rpm = data.get("M2", 0) / 10.0
                    m3_rpm = data.get("M3", 0) / 10.0
                    m4_rpm = data.get("M4", 0) / 10.0
                    
                    # Assign to wheel velocities matching joint state order:
                    # [0] = front_left (M1), [1] = front_right (M2), 
                    # [2] = rear_left (M4), [3] = rear_right (M3)
                    self.robot_state.wheel_velocities[0] = (m1_rpm / 60.0) * 2 * math.pi
                    self.robot_state.wheel_velocities[1] = (m2_rpm / 60.0) * 2 * math.pi
                    self.robot_state.wheel_velocities[2] = (m4_rpm / 60.0) * 2 * math.pi  # rear left
                    self.robot_state.wheel_velocities[3] = (m3_rpm / 60.0) * 2 * math.pi  # rear right
                
                # Parse odometry data (odl, odr) in cm
                if "odl" in data and "odr" in data:
                    odl_cm = data["odl"]
                    odr_cm = data["odr"]
                    # Convert cm to meters and then to wheel positions in radians
                    # position_rad = distance_m / wheel_radius
                    # odl = left wheel distance, odr = right wheel distance
                    # Assign to match joint state order:
                    # [0] = front_left (left), [1] = front_right (right),
                    # [2] = rear_left (left), [3] = rear_right (right)
                    self.robot_state.wheel_positions[0] = (odl_cm / 100.0) / self.WHEEL_RADIUS  # front left
                    self.robot_state.wheel_positions[1] = (odr_cm / 100.0) / self.WHEEL_RADIUS  # front right
                    self.robot_state.wheel_positions[2] = (odl_cm / 100.0) / self.WHEEL_RADIUS  # rear left
                    self.robot_state.wheel_positions[3] = (odr_cm / 100.0) / self.WHEEL_RADIUS  # rear right
                
                # Parse battery voltage (unit may vary by firmware version)
                if "v" in data:
                    # Assuming the value is in decivolts or similar small unit
                    # Example: v=1173 represents 11.73V
                    self.robot_state.battery_voltage = data["v"] / 100.0
                
                self.update_odometry()
            
            # Legacy format support (for backwards compatibility)
            # Update encoder data
            elif "encoder" in data and len(data["encoder"]) == 4:
                counts_per_rev = 1440
                for i, count in enumerate(data["encoder"]):
                    self.robot_state.wheel_positions[i] = (
                        (count / counts_per_rev) * 2 * math.pi
                    )
                
                # Update velocities
                if "velocity" in data and len(data["velocity"]) == 4:
                    self.robot_state.wheel_velocities = data["velocity"]
                
                # Update IMU
                if "imu" in data:
                    imu = data["imu"]
                    if "gyro" in imu:
                        self.robot_state.imu_angular_velocity = imu["gyro"]
                    if "accel" in imu:
                        self.robot_state.imu_linear_acceleration = imu["accel"]
                    if "orientation" in imu:
                        self.robot_state.imu_orientation = imu["orientation"]
                
                # Battery
                if "battery" in data:
                    self.robot_state.battery_voltage = data["battery"]
                
                self.update_odometry()
            
        except json.JSONDecodeError:
            self.get_logger().debug(f'Invalid JSON: {json_str[:50]}')
        except Exception as e:
            self.get_logger().error(f'Error parsing response: {e}')
    
    def update_odometry(self):
        """Calculate odometry from wheel encoders"""
        # Average left and right wheels
        left_vel = (self.robot_state.wheel_velocities[0] + 
                   self.robot_state.wheel_velocities[2]) / 2.0
        right_vel = (self.robot_state.wheel_velocities[1] + 
                    self.robot_state.wheel_velocities[3]) / 2.0
        
        # Differential drive kinematics
        linear_vel = (left_vel + right_vel) / 2.0 * self.WHEEL_RADIUS
        angular_vel = (right_vel - left_vel) * self.WHEEL_RADIUS / self.WHEEL_BASE
        
        # Integrate position
        dt = 0.02  # 50 Hz
        self.robot_state.position_x += (
            linear_vel * math.cos(self.robot_state.orientation_z) * dt
        )
        self.robot_state.position_y += (
            linear_vel * math.sin(self.robot_state.orientation_z) * dt
        )
        self.robot_state.orientation_z += angular_vel * dt
        
        # Normalise angle
        self.robot_state.orientation_z = math.atan2(
            math.sin(self.robot_state.orientation_z),
            math.cos(self.robot_state.orientation_z)
        )
        
        self.robot_state.linear_velocity = linear_vel
        self.robot_state.angular_velocity = angular_vel
    
    def publish_state_timer(self):
        """Publish all ROS2 topics"""
        stamp = self.get_clock().now()
        self.publish_odometry(stamp)
        self.publish_imu(stamp)
        self.publish_joint_states(stamp)
    
    def publish_odometry(self, stamp):
        """Publish odometry"""
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        
        odom.pose.pose.position.x = self.robot_state.position_x
        odom.pose.pose.position.y = self.robot_state.position_y
        odom.pose.pose.position.z = 0.0
        
        quat = self.yaw_to_quaternion(self.robot_state.orientation_z)
        odom.pose.pose.orientation = quat
        
        odom.twist.twist.linear.x = self.robot_state.linear_velocity
        odom.twist.twist.angular.z = self.robot_state.angular_velocity
        
        # Covariances
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[35] = 0.05
        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[35] = 0.05
        
        self.odom_pub.publish(odom)
        
        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.robot_state.position_x
            t.transform.translation.y = self.robot_state.position_y
            t.transform.translation.z = 0.0
            t.transform.rotation = quat
            self.tf_broadcaster.sendTransform(t)
    
    def publish_imu(self, stamp):
        """Publish IMU data"""
        imu = Imu()
        imu.header.stamp = stamp.to_msg()
        imu.header.frame_id = 'imu_link'
        
        imu.orientation.x = self.robot_state.imu_orientation[0]
        imu.orientation.y = self.robot_state.imu_orientation[1]
        imu.orientation.z = self.robot_state.imu_orientation[2]
        imu.orientation.w = self.robot_state.imu_orientation[3]
        
        imu.angular_velocity.x = self.robot_state.imu_angular_velocity[0]
        imu.angular_velocity.y = self.robot_state.imu_angular_velocity[1]
        imu.angular_velocity.z = self.robot_state.imu_angular_velocity[2]
        
        imu.linear_acceleration.x = self.robot_state.imu_linear_acceleration[0]
        imu.linear_acceleration.y = self.robot_state.imu_linear_acceleration[1]
        imu.linear_acceleration.z = self.robot_state.imu_linear_acceleration[2]
        
        # Covariances
        for i in [0, 4, 8]:
            imu.orientation_covariance[i] = 0.01
            imu.angular_velocity_covariance[i] = 0.01
            imu.linear_acceleration_covariance[i] = 0.01
        
        self.imu_pub.publish(imu)
    
    def publish_joint_states(self, stamp):
        """Publish wheel joint states"""
        js = JointState()
        js.header.stamp = stamp.to_msg()
        js.header.frame_id = self.base_frame
        
        js.name = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint'
        ]
        
        js.position = self.robot_state.wheel_positions
        js.velocity = self.robot_state.wheel_velocities
        
        self.joint_state_pub.publish(js)
    
    def watchdog_timer(self):
        """Safety watchdog - stop if no commands"""
        time_since_cmd = (
            self.get_clock().now() - self.last_cmd_time
        ).nanoseconds / 1e9
        
        if time_since_cmd > self.cmd_vel_timeout:
            self.send_json_command({"T": 1, "L": 0, "R": 0})
    
    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        """Convert yaw to quaternion"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q
    
    def destroy_node(self):
        """Cleanup"""
        if self.serial_conn and self.serial_conn.is_open:
            self.send_json_command({"T": 1, "L": 0, "R": 0})
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = CobraFlexDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
