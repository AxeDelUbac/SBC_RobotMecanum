#!/usr/bin/env python3
"""
Node de monitoring distant en mode console (sans GUI)
À utiliser si tkinter n'est pas disponible
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
import threading
import sys
import select


class ConsoleMonitor(Node):
    def __init__(self):
        super().__init__('console_monitor')
        
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
        self.last_status_time = 0
        
        # Interface console
        self.setup_console_interface()
        
        self.get_logger().info('Monitoring console démarré')
        self.print_help()
    
    def setup_console_interface(self):
        """Configure l'interface console"""
        # Thread pour la saisie utilisateur
        self.input_thread = threading.Thread(target=self.handle_input, daemon=True)
        self.input_thread.start()
        
        # Timer pour affichage périodique du statut
        self.status_timer = self.create_timer(5.0, self.print_status_summary)
    
    def handle_input(self):
        """Gère les commandes utilisateur en console"""
        while True:
            try:
                # Attendre l'entrée utilisateur avec timeout
                if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                    command = input().strip().lower()
                    self.process_command(command)
            except (EOFError, KeyboardInterrupt):
                break
            except Exception as e:
                print(f"Erreur saisie: {e}")
    
    def process_command(self, command):
        """Traite les commandes utilisateur"""
        if command == 'help' or command == 'h':
            self.print_help()
        
        elif command == 'status' or command == 's':
            self.print_detailed_status()
        
        elif command == 'stop':
            self.send_velocity(0.0, 0.0, 0.0)
            print("🛑 Robot arrêté")
        
        elif command.startswith('move'):
            parts = command.split()
            if len(parts) >= 4:
                try:
                    vx = float(parts[1])
                    vy = float(parts[2])
                    vz = float(parts[3])
                    self.send_velocity(vx, vy, vz)
                    print(f"🚀 Commande envoyée: vx={vx}, vy={vy}, vz={vz}")
                except ValueError:
                    print("❌ Format: move <vx> <vy> <vz>")
            else:
                print("❌ Format: move <vx> <vy> <vz>")
        
        elif command.startswith('mode'):
            parts = command.split()
            if len(parts) == 2:
                self.set_robot_mode(parts[1])
            else:
                print("❌ Format: mode <manual|auto|scan|stop>")
        
        elif command == 'forward' or command == 'f':
            self.send_velocity(0.2, 0.0, 0.0)
            print("⬆️ Avance")
        
        elif command == 'backward' or command == 'b':
            self.send_velocity(-0.2, 0.0, 0.0)
            print("⬇️ Recule")
        
        elif command == 'left' or command == 'l':
            self.send_velocity(0.0, 0.2, 0.0)
            print("⬅️ Gauche")
        
        elif command == 'right' or command == 'r':
            self.send_velocity(0.0, -0.2, 0.0)
            print("➡️ Droite")
        
        elif command == 'rotate_left' or command == 'rl':
            self.send_velocity(0.0, 0.0, 0.5)
            print("↺ Rotation gauche")
        
        elif command == 'rotate_right' or command == 'rr':
            self.send_velocity(0.0, 0.0, -0.5)
            print("↻ Rotation droite")
        
        elif command == 'quit' or command == 'q':
            print("👋 Au revoir!")
            rclpy.shutdown()
        
        else:
            print(f"❓ Commande inconnue: {command}")
            print("Tapez 'help' pour voir les commandes disponibles")
    
    def print_help(self):
        """Affiche l'aide des commandes"""
        print("\n" + "="*50)
        print("🤖 MONITORING ROBOT MECANUM - CONSOLE")
        print("="*50)
        print("Commandes disponibles:")
        print("  help, h           - Afficher cette aide")
        print("  status, s         - Statut détaillé du robot")
        print("  stop              - Arrêter le robot")
        print("  move <vx> <vy> <vz> - Mouvement personnalisé")
        print("  mode <mode>       - Changer le mode (manual/auto/scan/stop)")
        print("")
        print("Mouvements rapides:")
        print("  forward, f        - Avancer")
        print("  backward, b       - Reculer") 
        print("  left, l           - Aller à gauche")
        print("  right, r          - Aller à droite")
        print("  rotate_left, rl   - Tourner à gauche")
        print("  rotate_right, rr  - Tourner à droite")
        print("")
        print("  quit, q           - Quitter")
        print("="*50)
        print("Tapez une commande et appuyez sur Entrée...")
    
    def print_status_summary(self):
        """Affiche un résumé du statut périodiquement"""
        if self.last_status:
            current_time = time.time()
            if current_time - self.last_status_time > 2:  # Si pas de mise à jour depuis 2s
                print("⚠️  Pas de nouvelles du robot...")
            else:
                status_emoji = "🟢" if self.last_status.status == "idle" else "🔵"
                lidar_emoji = "📡" if self.last_status.lidar_active else "📵"
                micro_emoji = "🔗" if self.last_status.microcontroller_connected else "❌"
                
                print(f"\r{status_emoji} {self.last_status.robot_id} | "
                      f"Mode: {self.last_status.status} | "
                      f"Batterie: {self.last_status.battery_voltage:.1f}V | "
                      f"{lidar_emoji} {micro_emoji}", end="", flush=True)
    
    def print_detailed_status(self):
        """Affiche le statut détaillé"""
        if self.last_status:
            print("\n" + "-"*40)
            print("📊 STATUT DÉTAILLÉ DU ROBOT")
            print("-"*40)
            print(f"🤖 Robot ID: {self.last_status.robot_id}")
            print(f"📍 Position: ({self.last_status.pose.x:.2f}, {self.last_status.pose.y:.2f})")
            print(f"🔋 Batterie: {self.last_status.battery_voltage:.1f}V")
            print(f"🌡️  CPU: {self.last_status.cpu_temperature:.1f}°C")
            print(f"📡 Lidar: {'✅ Actif' if self.last_status.lidar_active else '❌ Inactif'}")
            print(f"🔗 Microcontrôleur: {'✅ Connecté' if self.last_status.microcontroller_connected else '❌ Déconnecté'}")
            print(f"🎯 Mode: {self.last_status.status}")
            if self.last_status.error_message:
                print(f"🚨 Erreur: {self.last_status.error_message}")
            print("-"*40)
        else:
            print("❌ Aucun statut reçu du robot")
    
    def status_callback(self, msg):
        """Traite les messages de statut du robot"""
        self.last_status = msg
        self.last_status_time = time.time()
        
        # Afficher les erreurs immédiatement
        if msg.error_message:
            print(f"\n🚨 ERREUR: {msg.error_message}")
    
    def sensor_callback(self, msg):
        """Traite les données des capteurs"""
        self.last_sensor_data = msg
        
        # Afficher les obstacles proches
        if msg.lidar_scan.ranges:
            min_range = min([r for r in msg.lidar_scan.ranges if r > 0 and r < 10])
            if min_range < 0.3:
                print(f"\n⚠️  Obstacle proche: {min_range:.2f}m")
    
    def send_velocity(self, linear_x, linear_y, angular_z):
        """Envoie une commande de vitesse"""
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.angular.z = angular_z
        
        self.cmd_vel_publisher.publish(msg)
    
    def set_robot_mode(self, mode):
        """Change le mode du robot"""
        if not self.mode_client.wait_for_service(timeout_sec=1.0):
            print("❌ Service de mode non disponible")
            return
        
        request = SetRobotMode.Request()
        request.mode = mode
        
        future = self.mode_client.call_async(request)
        future.add_done_callback(lambda f: self.mode_response_callback(f, mode))
    
    def mode_response_callback(self, future, requested_mode):
        """Traite la réponse du changement de mode"""
        try:
            response = future.result()
            if response.success:
                print(f"✅ Mode changé vers '{requested_mode}'")
            else:
                print(f"❌ Erreur changement mode: {response.message}")
        except Exception as e:
            print(f"❌ Erreur service mode: {e}")


def main(args=None):
    rclpy.init(args=args)
    console_monitor = ConsoleMonitor()
    
    try:
        rclpy.spin(console_monitor)
    except KeyboardInterrupt:
        print("\n👋 Arrêt du monitoring console")
    finally:
        console_monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()