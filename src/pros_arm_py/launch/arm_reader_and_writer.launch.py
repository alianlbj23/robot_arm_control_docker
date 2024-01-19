from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from pros_arm_py.env import ARM_SERIAL_PORT_DEFAULT
print("ARM_SERIAL_PORT_DEFAULT:",ARM_SERIAL_PORT_DEFAULT)
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port', 
            default_value=ARM_SERIAL_PORT_DEFAULT, 
            description='Serial port for the arm nodes'
        ),
        Node(
            package='pros_arm_py',  # Replace with your package name
            executable='arm_reader',
            name='arm_reader_node',
            parameters=[{'serial_port': LaunchConfiguration('serial_port')}],
            output='screen'
        ),
        Node(
            package='pros_arm_py',  # Replace with your package name
            executable='arm_writer',
            name='arm_writer_node',
            parameters=[{'serial_port': LaunchConfiguration('serial_port')}],
            output='screen'
        )
    ])
