#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests
import json
url = "http://192.168.1.214:8901/api/data/1"

class GPSSubcriberNode(Node):
    def __init__(self):
        super().__init__("gps_subcriber_node")
        self.create_subscription(String, 'gps_topic',self.gps_listener_callback,10)
        
    def gps_listener_callback(self,msg):
        # self.get_logger().info(f'GPS Data >> {msg.data}')
        gps_data = json.loads(msg.data)
        latitude = gps_data['latitude']
        longitude = gps_data['longitude']
        altitude = gps_data['altitude']
        speed = gps_data['speed']
        satellites = gps_data['satellites']
        payload = {
            "longitude": str(longitude),
            "latitude": str(latitude),
        }
        response = requests.patch(url,json=payload)
        if response.status_code == 200:
            print(response.json())
        else:
            print("Có lỗi xảy ra:", response.json())
        

def main(args=None):
    rclpy.init(args=args)
    node = GPSSubcriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()