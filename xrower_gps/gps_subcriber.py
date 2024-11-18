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

    def connect_to_server(self):
        try:
            sio.connect("http://192.168.1.214:8901")
            if sio.connected:
                self.get_logger().info("Connected to Socket.IO server.")
            else:
                self.get_logger().warn("Failed to connect to Socket.IO server, continuing without connection.")
        except socketio.exceptions.ConnectionError as e:
            self.get_logger().error(f"ConnectionError: {e}. Continuing without connection.")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}. Continuing without connection.")

def main(args=None):
    rclpy.init(args=args)
    node = GPSSubcriberNode()
    node.connect_to_server()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()