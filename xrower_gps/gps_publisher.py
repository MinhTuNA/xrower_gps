#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .tinygps.NeoM8N import get_gps_data
import json

class GPSPublisherNode(Node):
    
    def __init__(self):
        super().__init__("gps_publisher_node")
        self.publisher_ = self.create_publisher(String, 'gps_topic', 10)
        self.count = 0
        self.create_timer(0.5,self.gps_callback)
        
    def gps_callback(self):
        gps_data = get_gps_data()
        if gps_data is None:
            self.get_logger().warn("Failed to retrieve GPS data.")
        else: 
            msg = String()
            msg.data = json.dumps(gps_data)
            self.get_logger().info(f"\nLatitude: {gps_data['latitude']}\n"
                +f"Longitude: {gps_data['longitude']}\n"
                +f"Altitude: {gps_data['altitude']}\n"
                +f"Speed: {gps_data['speed']}\n"
                +f"Satellites: {gps_data['satellites']}\n")
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()