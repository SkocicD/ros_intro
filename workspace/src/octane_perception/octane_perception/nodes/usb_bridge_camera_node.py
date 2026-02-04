#!/usr/bin/env python3
"""
USB Bridge Camera Node
Receives camera data from USB bridge server and publishes to ROS topics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
import socket
import struct
import threading
import numpy as np


class USBBridgeCameraNode(Node):
    """Receives camera data via USB bridge and publishes to ROS topics"""

    def __init__(self):
        super().__init__('usb_bridge_camera_node')
        self.get_logger().info('USB Bridge Camera node started')

        # Publishers
        self.depth_publisher = self.create_publisher(Image, 'depth_camera/depth', 10)
        self.color_publisher = self.create_publisher(Image, 'depth_camera/color', 10)

        # Connection parameters
        self.declare_parameter('bridge_host', 'host.docker.internal')
        self.declare_parameter('bridge_port', 5555)

        self.bridge_host = self.get_parameter('bridge_host').value
        self.bridge_port = self.get_parameter('bridge_port').value

        # Socket connection
        self.socket = None
        self.running = True
        self.connected = False

        # Start connection thread
        self.connection_thread = threading.Thread(target=self.connection_loop)
        self.connection_thread.daemon = True
        self.connection_thread.start()

    def connect_to_bridge(self):
        """Connect to the USB bridge server"""
        try:
            self.get_logger().info(f'Connecting to USB bridge at {self.bridge_host}:{self.bridge_port}')
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.bridge_host, self.bridge_port))
            self.connected = True
            self.get_logger().info('Connected to USB bridge')
            return True
        except Exception as e:
            self.get_logger().warn(f'Failed to connect: {e}')
            return False

    def receive_frame(self):
        """Receive a frame from the bridge"""
        try:
            # Read header: device_type (1 byte) + data_length (4 bytes)
            header = self.socket.recv(5)
            if len(header) < 5:
                return None, None

            device_type, data_length = struct.unpack('!BI', header)

            # Read frame data
            data = b''
            while len(data) < data_length:
                chunk = self.socket.recv(min(4096, data_length - len(data)))
                if not chunk:
                    return None, None
                data += chunk

            return 'depth' if device_type == 0 else 'rgb', data

        except Exception as e:
            self.get_logger().error(f'Error receiving frame: {e}')
            return None, None

    def publish_frame(self, device_type, data):
        """Publish received frame as ROS Image message"""
        # Create Image message
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'

        # Parse image data based on device type
        if device_type == 'depth':
            # Depth: 640x480, 16-bit grayscale (2 bytes per pixel)
            msg.height = 480
            msg.width = 640
            msg.encoding = '16UC1'  # 16-bit unsigned, 1 channel
            msg.step = 640 * 2  # 2 bytes per pixel
            msg.data = list(data)
            self.depth_publisher.publish(msg)
            self.get_logger().info('Published depth frame', once=True)
        else:
            # Color: 640x480, 8-bit RGB (3 bytes per pixel)
            msg.height = 480
            msg.width = 640
            msg.encoding = 'rgb8'  # 8-bit RGB
            msg.step = 640 * 3  # 3 bytes per pixel
            msg.data = list(data)
            self.color_publisher.publish(msg)
            self.get_logger().info('Published color frame', once=True)

    def connection_loop(self):
        """Main connection and receive loop"""
        while self.running:
            if not self.connected:
                if self.connect_to_bridge():
                    # Connected, start receiving
                    while self.running and self.connected:
                        device_type, data = self.receive_frame()
                        if device_type and data:
                            self.publish_frame(device_type, data)
                        else:
                            # Connection lost
                            self.connected = False
                            self.get_logger().warn('Connection lost, reconnecting...')
                            if self.socket:
                                self.socket.close()
                            break
                else:
                    # Failed to connect, retry after delay
                    import time
                    time.sleep(2)
            else:
                import time
                time.sleep(0.1)

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.running = False
        self.connected = False
        if self.socket:
            self.socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = USBBridgeCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
