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

class GPSSubscriberNode(Node):
    def __init__(self):
        super().__init__("gps_subscriber_node")
        self.create_subscription(String, 'gps_topic',self.gps_listener_callback,10)
        
    def gps_listener_callback(self,msg):
        data = json.loads(msg.data)
        gps_data = data.get("gps", {})
        latitude = gps_data.get('latitude', 'N/A')
        longitude = gps_data.get('longitude', 'N/A')
        altitude = gps_data.get('altitude', 'N/A')
        speed = gps_data.get('speed', 'N/A')
        satellites = gps_data.get('satellites', 'N/A')

        compass_data = data.get("compass", {})
        heading = compass_data.get('heading', 'N/A')
        direction = compass_data.get('direction', 'N/A')
        
        self.get_logger().info(f"\nGPS Data:\n"
                               f"Latitude: {latitude}\n"
                               f"Longitude: {longitude}\n"
                               f"Altitude: {altitude}\n"
                               f"Speed: {speed}\n"
                               f"Satellites: {satellites}\n"
                               f"Compass Data:\n"
                               f"Heading: {heading:.2f}°\n"
                               f"Direction: {direction}")

        if sio.connected:
            sio.emit("DataFromROS2",{
                "latitude": latitude,
                "longitude": longitude,
                "altitude": altitude,
                "heading ": heading,
                "direction": direction,
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
    node = GPSSubscriberNode()
    node.connect_to_server()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()