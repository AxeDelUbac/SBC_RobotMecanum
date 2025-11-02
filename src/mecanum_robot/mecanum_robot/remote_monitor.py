#!/usr/bin/env python3
"""
Node de monitoring distant - à exécuter sur la machine de monitoring
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from mecanum_robot.msg import RobotStatus, SensorData
from mecanum_robot.srv import SetRobotMode
import json
import time
import tkinter as tk
from tkinter import ttk, scrolledtext
import threading


class RemoteMonitor(Node):
    def __init__(self):
        super().__init__('remote_monitor')
        
        # Paramètres
        self.declare_parameters(
            namespace='',
            parameters=[
                ('monitoring_ip', '192.168.1.100'),
                ('robot_ip', '192.168.1.101'),
            ]
        )
        
        self.monitoring_ip = self.get_parameter('monitoring_ip').value
        self.robot_ip = self.get_parameter('robot_ip').value
        
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
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Subscribers
        self.status_subscriber = self.create_subscription(
            RobotStatus, 'robot_status', self.status_callback, reliable_qos)
        self.sensor_subscriber = self.create_subscription(
            SensorData, 'sensor_data', self.sensor_callback, sensor_qos)
        
        # Service clients
        self.mode_client = self.create_client(SetRobotMode, 'set_robot_mode')
        
        # État
        self.last_status = None
        self.last_sensor_data = None
        
        # Interface utilisateur
        self.setup_gui()
        
        self.get_logger().info('Monitoring distant démarré')
    
    def setup_gui(self):
        """Configure l'interface graphique de monitoring"""
        self.root = tk.Tk()
        self.root.title("Robot Mecanum - Monitoring")
        self.root.geometry("800x600")
        
        # Frame principale
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Status du robot
        status_frame = ttk.LabelFrame(main_frame, text="Statut du Robot")
        status_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.status_labels = {}
        status_items = [
            ('Robot ID:', 'robot_id'),
            ('Statut:', 'status'),
            ('Position X:', 'pose_x'),
            ('Position Y:', 'pose_y'),
            ('Batterie:', 'battery'),
            ('Température CPU:', 'cpu_temp'),
            ('Lidar:', 'lidar_status'),
            ('Microcontrôleur:', 'micro_status')
        ]
        
        for i, (label_text, key) in enumerate(status_items):
            ttk.Label(status_frame, text=label_text).grid(
                row=i//2, column=(i%2)*2, sticky=tk.W, padx=5, pady=2)
            self.status_labels[key] = ttk.Label(status_frame, text="N/A")
            self.status_labels[key].grid(
                row=i//2, column=(i%2)*2+1, sticky=tk.W, padx=5, pady=2)
        
        # Contrôles
        control_frame = ttk.LabelFrame(main_frame, text="Contrôles")
        control_frame.pack(fill=tk.X, pady=(0, 10))
        
        # Modes
        ttk.Label(control_frame, text="Mode:").grid(row=0, column=0, padx=5, pady=5)
        self.mode_var = tk.StringVar(value="idle")
        mode_combo = ttk.Combobox(control_frame, textvariable=self.mode_var,
                                  values=["manual", "auto", "scan", "stop"])
        mode_combo.grid(row=0, column=1, padx=5, pady=5)
        
        ttk.Button(control_frame, text="Appliquer Mode",
                   command=self.set_robot_mode).grid(row=0, column=2, padx=5, pady=5)
        
        # Contrôle manuel
        manual_frame = ttk.LabelFrame(control_frame, text="Contrôle Manuel")
        manual_frame.grid(row=1, column=0, columnspan=3, sticky=tk.EW, padx=5, pady=5)
        
        # Boutons directionnels
        ttk.Button(manual_frame, text="↑", 
                   command=lambda: self.send_velocity(0.2, 0.0, 0.0)).grid(row=0, column=1)
        ttk.Button(manual_frame, text="←", 
                   command=lambda: self.send_velocity(0.0, 0.2, 0.0)).grid(row=1, column=0)
        ttk.Button(manual_frame, text="STOP", 
                   command=lambda: self.send_velocity(0.0, 0.0, 0.0)).grid(row=1, column=1)
        ttk.Button(manual_frame, text="→", 
                   command=lambda: self.send_velocity(0.0, -0.2, 0.0)).grid(row=1, column=2)
        ttk.Button(manual_frame, text="↓", 
                   command=lambda: self.send_velocity(-0.2, 0.0, 0.0)).grid(row=2, column=1)
        
        # Rotation
        ttk.Button(manual_frame, text="↺", 
                   command=lambda: self.send_velocity(0.0, 0.0, 0.5)).grid(row=0, column=0)
        ttk.Button(manual_frame, text="↻", 
                   command=lambda: self.send_velocity(0.0, 0.0, -0.5)).grid(row=0, column=2)
        
        # Logs
        log_frame = ttk.LabelFrame(main_frame, text="Logs")
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        self.log_text = scrolledtext.ScrolledText(log_frame, height=15)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Démarrer la GUI dans un thread séparé
        self.gui_thread = threading.Thread(target=self.run_gui, daemon=True)
        self.gui_thread.start()
    
    def run_gui(self):
        """Exécute la boucle principale de la GUI"""
        self.root.mainloop()
    
    def log_message(self, message):
        """Ajoute un message aux logs"""
        timestamp = time.strftime("%H:%M:%S")
        log_entry = f"[{timestamp}] {message}\n"
        self.log_text.insert(tk.END, log_entry)
        self.log_text.see(tk.END)
    
    def status_callback(self, msg):
        """Traite les messages de statut du robot"""
        self.last_status = msg
        
        # Mettre à jour l'interface
        if hasattr(self, 'status_labels'):
            self.status_labels['robot_id'].config(text=msg.robot_id)
            self.status_labels['status'].config(text=msg.status)
            self.status_labels['pose_x'].config(text=f"{msg.pose.x:.2f} m")
            self.status_labels['pose_y'].config(text=f"{msg.pose.y:.2f} m")
            self.status_labels['battery'].config(text=f"{msg.battery_voltage:.1f} V")
            self.status_labels['cpu_temp'].config(text=f"{msg.cpu_temperature:.1f} °C")
            self.status_labels['lidar_status'].config(
                text="Actif" if msg.lidar_active else "Inactif")
            self.status_labels['micro_status'].config(
                text="Connecté" if msg.microcontroller_connected else "Déconnecté")
            
            if msg.error_message:
                self.log_message(f"ERREUR: {msg.error_message}")
    
    def sensor_callback(self, msg):
        """Traite les données des capteurs"""
        self.last_sensor_data = msg
        
        # Log des informations capteurs
        if msg.lidar_scan.ranges:
            min_range = min([r for r in msg.lidar_scan.ranges if r > 0])
            self.log_message(f"Lidar - Distance min: {min_range:.2f}m")
    
    def send_velocity(self, linear_x, linear_y, angular_z):
        """Envoie une commande de vitesse"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        
        self.cmd_vel_publisher.publish(msg)
        self.log_message(f"Commande: vx={linear_x}, vy={linear_y}, wz={angular_z}")
    
    def set_robot_mode(self):
        """Change le mode du robot"""
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            self.log_message("Service de mode non disponible")
            return
        
        request = SetRobotMode.Request()
        request.mode = self.mode_var.get()
        
        future = self.mode_client.call_async(request)
        future.add_done_callback(self.mode_response_callback)
    
    def mode_response_callback(self, future):
        """Traite la réponse du changement de mode"""
        try:
            response = future.result()
            if response.success:
                self.log_message(f"Mode changé: {response.message}")
            else:
                self.log_message(f"Erreur mode: {response.message}")
        except Exception as e:
            self.log_message(f"Erreur service mode: {e}")


def main(args=None):
    rclpy.init(args=args)
    remote_monitor = RemoteMonitor()
    
    try:
        rclpy.spin(remote_monitor)
    except KeyboardInterrupt:
        pass
    finally:
        remote_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()