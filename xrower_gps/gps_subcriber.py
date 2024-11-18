#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

import socketio

sio = socketio.Client()


@sio.event
def connect():
    print("Connected to server")

@sio.event
def message(data):
    print("Received from server:", data)

@sio.event
def disconnect():
    print("Disconnected from server")

class GPSSubcriberNode(Node):
    def __init__(self):
        super().__init__("gps_subcriber_node")
        self.create_subscription(String, 'gps_topic',self.gps_listener_callback,10)
        
    def gps_listener_callback(self,msg):
        gps_data = json.loads(msg.data)
        latitude = gps_data['latitude']
        longitude = gps_data['longitude']
        altitude = gps_data['altitude']
        speed = gps_data['speed']
        satellites = gps_data['satellites']
        self.get_logger().info(msg.data)
        if sio.connected:
            sio.emit("DataFromROS2",{
                "latitude": latitude,
                "longitude": longitude,
            })

def main(args=None):
    rclpy.init(args=args)
    node = GPSSubcriberNode()
    sio.connect("http://192.168.1.214:8901")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()