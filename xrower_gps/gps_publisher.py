#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .tinygps.NeoM8N import get_gps_data
from .ist8310 import reset_ist8310, initialize_ist8310,get_ist8310_data
import json

class GPSPublisherNode(Node):
    
    def __init__(self):
        super().__init__("gps_publisher_node")
        self.publisher_ = self.create_publisher(String, 'gps_topic', 10)
        reset_ist8310()
        if not initialize_ist8310():
            self.get_logger().error("Failed to initialize IST8310 sensor.")
            rclpy.shutdown() 
        self.create_timer(0.5,self.gps_callback)
        
    def gps_callback(self):
        
        gps_data = get_gps_data()
        compass_data = get_ist8310_data()
        if gps_data is None or compass_data is None :
            self.get_logger().warn("Failed to retrieve GPS data or compass data")
        else: 
            data = {
                "gps": gps_data,
                "compass": compass_data
            }
            msg = String()
            msg.data = json.dumps(data)
            self.get_logger().info(f"\nGPS Data:\n"
                f"Latitude: {gps_data['latitude']}\n"
                f"Longitude: {gps_data['longitude']}\n"
                f"Altitude: {gps_data['altitude']}\n"
                f"Speed: {gps_data['speed']}\n"
                f"Satellites: {gps_data['satellites']}\n"
                f"Compass Data:\n"
                f"Heading: {compass_data['heading']:.2f}°\n"
                f"Direction: {compass_data['direction']}")
            self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GPSPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()