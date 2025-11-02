#!/usr/bin/env python3
"""
Bridge de communication entre machines pour ROS2
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from mecanum_robot.msg import RobotStatus, SensorData
import json
import socket
import threading
import time
import select


class CommunicationBridge(Node):
    def __init__(self):
        super().__init__('communication_bridge')
        
        # Paramètres
        self.declare_parameters(
            namespace='',
            parameters=[
                ('bridge_port', 11511),
                ('max_connections', 5),
                ('heartbeat_interval', 1.0),
                ('is_server', True),  # True sur RPi, False sur machine monitoring
            ]
        )
        
        self.bridge_port = self.get_parameter('bridge_port').value
        self.max_connections = self.get_parameter('max_connections').value
        self.heartbeat_interval = self.get_parameter('heartbeat_interval').value
        self.is_server = self.get_parameter('is_server').value
        
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
        
        # Publishers et Subscribers selon le mode
        if self.is_server:
            # Mode serveur (Raspberry Pi)
            self.setup_server_mode(sensor_qos, reliable_qos)
        else:
            # Mode client (Machine monitoring)
            self.setup_client_mode(sensor_qos, reliable_qos)
        
        # Communication réseau
        self.connected_clients = {}
        self.server_socket = None
        self.client_socket = None
        self.running = True
        
        # Démarrer la communication réseau
        self.start_network_communication()
        
        # Timer pour heartbeat
        self.heartbeat_timer = self.create_timer(
            self.heartbeat_interval, self.send_heartbeat)
        
        self.get_logger().info(f'Bridge de communication démarré en mode {"serveur" if self.is_server else "client"}')
    
    def setup_server_mode(self, sensor_qos, reliable_qos):
        """Configuration pour le mode serveur (Raspberry Pi)"""
        # Subscribers - écoute les topics locaux pour les transmettre
        self.status_subscriber = self.create_subscription(
            RobotStatus, 'robot_status', self.forward_status, reliable_qos)
        self.sensor_subscriber = self.create_subscription(
            SensorData, 'sensor_data', self.forward_sensor_data, sensor_qos)
        
        # Publishers - reçoit des commandes du réseau et les republique localement
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.network_cmd_publisher = self.create_publisher(String, 'network_commands', 10)
    
    def setup_client_mode(self, sensor_qos, reliable_qos):
        """Configuration pour le mode client (Machine monitoring)"""
        # Publishers - envoie les commandes vers le réseau
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, 'cmd_vel', self.forward_cmd_vel, 10)
        
        # Subscribers - reçoit les données du réseau et les republique localement
        self.status_publisher = self.create_publisher(RobotStatus, 'robot_status', reliable_qos)
        self.sensor_publisher = self.create_publisher(SensorData, 'sensor_data', sensor_qos)
    
    def start_network_communication(self):
        """Démarre la communication réseau"""
        if self.is_server:
            self.start_server()
        else:
            self.start_client()
    
    def start_server(self):
        """Démarre le serveur TCP"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', self.bridge_port))
            self.server_socket.listen(self.max_connections)
            self.server_socket.setblocking(False)
            
            self.get_logger().info(f'Serveur démarré sur le port {self.bridge_port}')
            
            # Thread pour accepter les connexions
            self.server_thread = threading.Thread(target=self.accept_connections, daemon=True)
            self.server_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'Erreur démarrage serveur: {e}')
    
    def start_client(self):
        """Démarre le client TCP"""
        # Thread pour la connexion client
        self.client_thread = threading.Thread(target=self.connect_to_server, daemon=True)
        self.client_thread.start()
    
    def accept_connections(self):
        """Accepte les connexions clients"""
        while self.running:
            try:
                ready, _, _ = select.select([self.server_socket], [], [], 1.0)
                if ready:
                    client_socket, address = self.server_socket.accept()
                    client_id = f"{address[0]}:{address[1]}"
                    
                    self.connected_clients[client_id] = {
                        'socket': client_socket,
                        'address': address,
                        'last_heartbeat': time.time()
                    }
                    
                    self.get_logger().info(f'Client connecté: {client_id}')
                    
                    # Thread pour gérer ce client
                    client_thread = threading.Thread(
                        target=self.handle_client, 
                        args=(client_id,), 
                        daemon=True
                    )
                    client_thread.start()
                    
            except Exception as e:
                if self.running:
                    self.get_logger().error(f'Erreur acceptation connexion: {e}')
    
    def connect_to_server(self):
        """Se connecte au serveur"""
        server_ip = "192.168.1.101"  # IP de la Raspberry Pi
        
        while self.running:
            try:
                self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.client_socket.connect((server_ip, self.bridge_port))
                self.get_logger().info(f'Connecté au serveur {server_ip}:{self.bridge_port}')
                
                # Gérer la réception des messages
                self.handle_server_messages()
                
            except Exception as e:
                self.get_logger().error(f'Erreur connexion serveur: {e}')
                time.sleep(5)  # Attendre avant de réessayer
    
    def handle_client(self, client_id):
        """Gère les messages d'un client"""
        client_info = self.connected_clients[client_id]
        client_socket = client_info['socket']
        
        try:
            while self.running and client_id in self.connected_clients:
                ready, _, _ = select.select([client_socket], [], [], 1.0)
                if ready:
                    data = client_socket.recv(4096)
                    if not data:
                        break
                    
                    self.process_received_message(data.decode('utf-8'))
                    client_info['last_heartbeat'] = time.time()
                    
        except Exception as e:
            self.get_logger().error(f'Erreur client {client_id}: {e}')
        finally:
            self.disconnect_client(client_id)
    
    def handle_server_messages(self):
        """Gère les messages du serveur (mode client)"""
        try:
            while self.running:
                ready, _, _ = select.select([self.client_socket], [], [], 1.0)
                if ready:
                    data = self.client_socket.recv(4096)
                    if not data:
                        break
                    
                    self.process_received_message(data.decode('utf-8'))
                    
        except Exception as e:
            self.get_logger().error(f'Erreur réception serveur: {e}')
        finally:
            if self.client_socket:
                self.client_socket.close()
    
    def process_received_message(self, message):
        """Traite un message reçu via le réseau"""
        try:
            data = json.loads(message)
            msg_type = data.get('type')
            
            if msg_type == 'cmd_vel' and self.is_server:
                # Republier la commande localement
                twist_msg = Twist()
                twist_msg.linear.x = data['linear_x']
                twist_msg.linear.y = data['linear_y']
                twist_msg.angular.z = data['angular_z']
                self.cmd_vel_publisher.publish(twist_msg)
                
            elif msg_type == 'robot_status' and not self.is_server:
                # Convertir et republier le statut
                # (Implémentation simplifiée - à adapter selon vos besoins)
                pass
                
            elif msg_type == 'heartbeat':
                # Heartbeat reçu
                pass
                
        except json.JSONDecodeError:
            self.get_logger().error('Message JSON invalide reçu')
        except Exception as e:
            self.get_logger().error(f'Erreur traitement message: {e}')
    
    def forward_status(self, msg):
        """Transmet le statut du robot via le réseau"""
        if self.is_server:
            data = {
                'type': 'robot_status',
                'robot_id': msg.robot_id,
                'status': msg.status,
                'pose_x': msg.pose.x,
                'pose_y': msg.pose.y,
                'pose_theta': msg.pose.theta,
                'battery_voltage': msg.battery_voltage,
                'cpu_temperature': msg.cpu_temperature,
                'lidar_active': msg.lidar_active,
                'microcontroller_connected': msg.microcontroller_connected,
                'timestamp': time.time()
            }
            self.broadcast_to_clients(json.dumps(data))
    
    def forward_sensor_data(self, msg):
        """Transmet les données des capteurs via le réseau"""
        if self.is_server:
            # Envoyer seulement un résumé des données lidar pour éviter la surcharge
            data = {
                'type': 'sensor_data',
                'lidar_ranges_count': len(msg.lidar_scan.ranges),
                'lidar_min_range': min(msg.lidar_scan.ranges) if msg.lidar_scan.ranges else 0,
                'encoder_values': list(msg.encoder_values),
                'sensor_status': msg.sensor_status,
                'timestamp': time.time()
            }
            self.broadcast_to_clients(json.dumps(data))
    
    def forward_cmd_vel(self, msg):
        """Transmet les commandes de vitesse via le réseau"""
        if not self.is_server and self.client_socket:
            data = {
                'type': 'cmd_vel',
                'linear_x': msg.linear.x,
                'linear_y': msg.linear.y,
                'angular_z': msg.angular.z,
                'timestamp': time.time()
            }
            try:
                self.client_socket.send(json.dumps(data).encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'Erreur envoi cmd_vel: {e}')
    
    def broadcast_to_clients(self, message):
        """Diffuse un message à tous les clients connectés"""
        disconnected_clients = []
        
        for client_id, client_info in self.connected_clients.items():
            try:
                client_info['socket'].send(message.encode('utf-8'))
            except Exception as e:
                self.get_logger().error(f'Erreur envoi à {client_id}: {e}')
                disconnected_clients.append(client_id)
        
        # Nettoyer les clients déconnectés
        for client_id in disconnected_clients:
            self.disconnect_client(client_id)
    
    def disconnect_client(self, client_id):
        """Déconnecte un client"""
        if client_id in self.connected_clients:
            try:
                self.connected_clients[client_id]['socket'].close()
            except:
                pass
            del self.connected_clients[client_id]
            self.get_logger().info(f'Client déconnecté: {client_id}')
    
    def send_heartbeat(self):
        """Envoie un heartbeat périodique"""
        heartbeat_data = {
            'type': 'heartbeat',
            'timestamp': time.time(),
            'node_id': self.get_name()
        }
        
        if self.is_server:
            self.broadcast_to_clients(json.dumps(heartbeat_data))
        elif self.client_socket:
            try:
                self.client_socket.send(json.dumps(heartbeat_data).encode('utf-8'))
            except:
                pass
    
    def destroy_node(self):
        """Nettoyage lors de la destruction du node"""
        self.running = False
        
        if self.server_socket:
            self.server_socket.close()
        
        if self.client_socket:
            self.client_socket.close()
        
        for client_info in self.connected_clients.values():
            client_info['socket'].close()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    communication_bridge = CommunicationBridge()
    
    try:
        rclpy.spin(communication_bridge)
    except KeyboardInterrupt:
        pass
    finally:
        communication_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()