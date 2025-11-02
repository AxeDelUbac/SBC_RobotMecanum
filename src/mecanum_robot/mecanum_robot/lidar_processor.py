#!/usr/bin/env python3
"""
Node de traitement des données lidar
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Point
from std_msgs.msg import Float32MultiArray
import numpy as np
import math


class LidarProcessor(Node):
    def __init__(self):
        super().__init__('lidar_processor')
        
        # Paramètres
        self.declare_parameters(
            namespace='',
            parameters=[
                ('min_obstacle_distance', 0.3),  # mètres
                ('max_scan_range', 5.0),         # mètres
                ('angle_increment', 1.0),        # degrés pour la réduction de données
            ]
        )
        
        self.min_obstacle_distance = self.get_parameter('min_obstacle_distance').value
        self.max_scan_range = self.get_parameter('max_scan_range').value
        self.angle_increment = self.get_parameter('angle_increment').value
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Subscribers
        self.lidar_subscriber = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, sensor_qos)
        
        # Publishers
        self.obstacles_publisher = self.create_publisher(
            Float32MultiArray, 'obstacles', 10)
        self.filtered_scan_publisher = self.create_publisher(
            LaserScan, 'scan_filtered', sensor_qos)
        
        self.get_logger().info('Processeur lidar démarré')
    
    def lidar_callback(self, msg):
        """Traite les données du lidar"""
        try:
            # Filtrer les données
            filtered_scan = self.filter_scan(msg)
            self.filtered_scan_publisher.publish(filtered_scan)
            
            # Détecter les obstacles
            obstacles = self.detect_obstacles(msg)
            self.publish_obstacles(obstacles)
            
            # Log des obstacles proches
            close_obstacles = [obs for obs in obstacles if obs['distance'] < self.min_obstacle_distance]
            if close_obstacles:
                self.get_logger().warn(f'Obstacles proches détectés: {len(close_obstacles)}')
        
        except Exception as e:
            self.get_logger().error(f'Erreur traitement lidar: {e}')
    
    def filter_scan(self, scan):
        """Filtre le scan lidar pour réduire le bruit"""
        filtered_scan = LaserScan()
        filtered_scan.header = scan.header
        filtered_scan.angle_min = scan.angle_min
        filtered_scan.angle_max = scan.angle_max
        filtered_scan.angle_increment = scan.angle_increment
        filtered_scan.time_increment = scan.time_increment
        filtered_scan.scan_time = scan.scan_time
        filtered_scan.range_min = scan.range_min
        filtered_scan.range_max = min(scan.range_max, self.max_scan_range)
        
        # Filtrer les valeurs aberrantes et appliquer un filtre médian simple
        ranges = np.array(scan.ranges)
        
        # Remplacer les valeurs infinies/NaN par range_max
        ranges = np.where(np.isfinite(ranges), ranges, filtered_scan.range_max)
        
        # Filtre médian sur une fenêtre de 3 points
        filtered_ranges = []
        for i in range(len(ranges)):
            window = []
            for j in range(max(0, i-1), min(len(ranges), i+2)):
                if scan.range_min <= ranges[j] <= filtered_scan.range_max:
                    window.append(ranges[j])
            
            if window:
                filtered_ranges.append(np.median(window))
            else:
                filtered_ranges.append(filtered_scan.range_max)
        
        filtered_scan.ranges = filtered_ranges
        filtered_scan.intensities = scan.intensities
        
        return filtered_scan
    
    def detect_obstacles(self, scan):
        """Détecte les obstacles dans le scan lidar"""
        obstacles = []
        
        for i, distance in enumerate(scan.ranges):
            if scan.range_min <= distance <= self.max_scan_range:
                # Calculer l'angle
                angle = scan.angle_min + i * scan.angle_increment
                
                # Convertir en coordonnées cartésiennes
                x = distance * math.cos(angle)
                y = distance * math.sin(angle)
                
                obstacle = {
                    'distance': distance,
                    'angle': angle,
                    'x': x,
                    'y': y,
                    'index': i
                }
                
                obstacles.append(obstacle)
        
        return obstacles
    
    def publish_obstacles(self, obstacles):
        """Publie la liste des obstacles détectés"""
        msg = Float32MultiArray()
        
        # Format: [distance1, angle1, x1, y1, distance2, angle2, x2, y2, ...]
        data = []
        for obs in obstacles:
            data.extend([obs['distance'], obs['angle'], obs['x'], obs['y']])
        
        msg.data = data
        self.obstacles_publisher.publish(msg)
    
    def get_sector_info(self, scan, start_angle, end_angle):
        """Obtient les informations d'un secteur angulaire"""
        start_index = int((start_angle - scan.angle_min) / scan.angle_increment)
        end_index = int((end_angle - scan.angle_min) / scan.angle_increment)
        
        start_index = max(0, start_index)
        end_index = min(len(scan.ranges), end_index)
        
        sector_ranges = scan.ranges[start_index:end_index]
        valid_ranges = [r for r in sector_ranges if scan.range_min <= r <= self.max_scan_range]
        
        if valid_ranges:
            return {
                'min_distance': min(valid_ranges),
                'avg_distance': np.mean(valid_ranges),
                'obstacle_count': len([r for r in valid_ranges if r < self.min_obstacle_distance])
            }
        else:
            return {
                'min_distance': self.max_scan_range,
                'avg_distance': self.max_scan_range,
                'obstacle_count': 0
            }


def main(args=None):
    rclpy.init(args=args)
    lidar_processor = LidarProcessor()
    
    try:
        rclpy.spin(lidar_processor)
    except KeyboardInterrupt:
        pass
    finally:
        lidar_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()