import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

# Publish depth and image data from Astra Pro
class AstraDepthNode(Node):

    def __init__(self):
        super().__init__('astra_depth_node')
        self.get_logger().info('Depth Camera node started')
        self.depth_publisher = self.create_publisher(Image, 'astra/depth/image_raw', 10)

def main(args=None):
    rclpy.init(args=args)
    node = AstraDepthNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()