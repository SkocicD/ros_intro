from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# Generate launch description for perception subsystem
def generate_launch_description():

    # Declare launch argument for USB bridge mode
    use_usb_bridge_arg = DeclareLaunchArgument(
        'use_usb_bridge',
        default_value='false',
        description='Use USB bridge for Mac development (true/false)'
    )

    use_usb_bridge = LaunchConfiguration('use_usb_bridge')

    # ========================================
    # Camera Driver (Native or USB Bridge)
    # ========================================

    # Native Orbbec camera driver (for Jetson)
    orbbec_camera_dir = get_package_share_directory('orbbec_camera')
    astra_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_camera_dir, 'launch', 'astra.launch.py')
        ),
        condition=UnlessCondition(use_usb_bridge)
    )

    # Native camera adapter node
    astra_depth_node = Node(
        package='octane_perception',
        executable='astra_depth_node',
        name='astra_depth_node',
        output='screen',
        condition=UnlessCondition(use_usb_bridge)
    )

    # USB bridge node (for Mac development)
    usb_bridge_node = Node(
        package='octane_perception',
        executable='usb_bridge_camera_node',
        name='usb_bridge_camera_node',
        output='screen',
        parameters=[{
            'bridge_host': 'host.docker.internal',
            'bridge_port': 5555
        }],
        condition=IfCondition(use_usb_bridge)
    )

    # ========================================
    # Perception Nodes
    # ========================================

    camera_viewer_node = Node(
        package='octane_perception',
        executable='camera_viewer_node',
        name='camera_viewer_node',
        output='screen',
    )

    return LaunchDescription([
        use_usb_bridge_arg,
        LogInfo(msg='Starting perception subsystem'),

        # Camera driver (native or bridge)
        astra_launch,
        astra_depth_node,
        usb_bridge_node,

        # Perception nodes
        camera_viewer_node,

        LogInfo(msg='Perception subsystem online'),
    ])
