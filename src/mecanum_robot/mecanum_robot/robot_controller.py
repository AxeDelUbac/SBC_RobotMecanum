#!/usr/bin/env python3
"""
Node contrôleur principal du robot - à exécuter sur la Raspberry Pi
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
from mecanum_robot.msg import RobotStatus, SensorData
from mecanum_robot.srv import SetRobotMode
import json
import socket
import threading
import time
import psutil
import serial
import subprocess


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        
        # Paramètres
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id', 'mecanum_robot_01'),
                ('microcontroller_port', '/dev/ttyACM0'),
                ('status_hz', 1.0),
                ('sensor_data_hz', 10.0),
            ]
        )
        
        self.robot_id = self.get_parameter('robot_id').value
        self.microcontroller_port = self.get_parameter('microcontroller_port').value
        self.status_hz = self.get_parameter('status_hz').value
        self.sensor_data_hz = self.get_parameter('sensor_data_hz').value
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )
        
        # Publishers
        self.status_publisher = self.create_publisher(
            RobotStatus, 'robot_status', reliable_qos)
        self.sensor_data_publisher = self.create_publisher(
            SensorData, 'sensor_data', sensor_qos)
        
        # Subscribers
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.lidar_subscriber = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, sensor_qos)
        
        # Services
        self.mode_service = self.create_service(
            SetRobotMode, 'set_robot_mode', self.set_mode_callback)
        
        # État du robot
        self.current_mode = "idle"
        self.pose = Pose2D()
        self.last_lidar_scan = None
        self.microcontroller_connected = False
        self.error_message = ""
        
        # Communication série avec microcontrôleur
        self.setup_microcontroller()
        
        # Timers
        self.status_timer = self.create_timer(
            1.0 / self.status_hz, self.publish_status)
        self.sensor_timer = self.create_timer(
            1.0 / self.sensor_data_hz, self.publish_sensor_data)
        
        self.get_logger().info(f'Robot controller started for {self.robot_id}')
    
    def setup_microcontroller(self):
        """Configure la communication avec le microcontrôleur"""
        try:
            self.serial_conn = serial.Serial(
                self.microcontroller_port, 
                baudrate=115200, 
                timeout=1
            )
            self.microcontroller_connected = True
            self.get_logger().info('Microcontrôleur connecté')
        except Exception as e:
            self.get_logger().error(f'Erreur connexion microcontrôleur: {e}')
            self.microcontroller_connected = False
            self.serial_conn = None
    
    def cmd_vel_callback(self, msg):
        """Traite les commandes de vitesse"""
        if self.microcontroller_connected and self.serial_conn:
            try:
                # Format: "VEL:linear_x,linear_y,angular_z\n"
                cmd = f"VEL:{msg.linear.x:.3f},{msg.linear.y:.3f},{msg.angular.z:.3f}\n"
                self.serial_conn.write(cmd.encode())
            except Exception as e:
                self.get_logger().error(f'Erreur envoi commande: {e}')
    
    def lidar_callback(self, msg):
        """Traite les données du lidar"""
        self.last_lidar_scan = msg
    
    def set_mode_callback(self, request, response):
        """Service pour changer le mode du robot"""
        valid_modes = ["manual", "auto", "scan", "stop"]
        if request.mode in valid_modes:
            self.current_mode = request.mode
            response.success = True
            response.message = f"Mode changé vers {request.mode}"
            self.get_logger().info(f'Mode changé: {request.mode}')
        else:
            response.success = False
            response.message = f"Mode invalide. Modes valides: {valid_modes}"
        
        return response
    
    def get_system_info(self):
        """Récupère les informations système"""
        try:
            battery_voltage = 12.0  # À adapter selon votre système
            cpu_temp = psutil.sensors_temperatures().get('cpu_thermal', [{'current': 0}])[0]['current']
            return battery_voltage, cpu_temp
        except:
            return 0.0, 0.0
    
    def publish_status(self):
        """Publie le statut du robot"""
        msg = RobotStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.robot_id = self.robot_id
        msg.status = self.current_mode
        msg.pose = self.pose
        
        battery_voltage, cpu_temp = self.get_system_info()
        msg.battery_voltage = battery_voltage
        msg.cpu_temperature = cpu_temp
        msg.lidar_active = self.last_lidar_scan is not None
        msg.microcontroller_connected = self.microcontroller_connected
        msg.active_sensors = ["lidar", "odometry"] if self.microcontroller_connected else ["lidar"]
        msg.error_message = self.error_message
        
        self.status_publisher.publish(msg)
    
    def publish_sensor_data(self):
        """Publie les données des capteurs"""
        if self.last_lidar_scan is None:
            return
            
        msg = SensorData()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.lidar_scan = self.last_lidar_scan
        
        # Données d'odométrie (à adapter selon votre microcontrôleur)
        msg.encoder_values = [0.0, 0.0, 0.0, 0.0]  # 4 roues mecanum
        msg.additional_sensor_data = []
        msg.sensor_status = "active"
        
        self.sensor_data_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    
    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        pass
    finally:
        robot_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()