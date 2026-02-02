from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

# Generate launch description for perception subsystem
def generate_launch_description():

    # Get Orbbec camera launch file for Astra Pro
    orbbec_camera_dir = get_package_share_directory('orbbec_camera')
    astra_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(orbbec_camera_dir, 'launch', 'astra.launch.py')
        )
    )

    # Launch octane_perception nodes
    astra_depth_node = Node(
        package='octane_perception',
        executable='astra_depth_node',
        name='astra_depth_node',
        output='screen',
    )

    camera_viewer_node = Node(
        package='octane_perception',
        executable='camera_viewer_node',
        name='camera_viewer_node',
        output='screen',
    )

    return LaunchDescription([
        LogInfo(msg='Starting perception subsystem'),
        astra_launch,
        astra_depth_node,
        camera_viewer_node,
        LogInfo(msg='Perception subsystem online'),
    ])
