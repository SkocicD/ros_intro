from setuptools import find_packages, setup

package_name = 'octane_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'astra_depth_node = octane_perception.nodes.astra_depth_node:main',
            'usb_bridge_camera_node = octane_perception.nodes.usb_bridge_camera_node:main',
            'rgb_camera_node = octane_perception.nodes.rgb_camera_node:main',
        ],
    },
)
