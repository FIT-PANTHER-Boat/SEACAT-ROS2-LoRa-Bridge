from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='ros2_lora',
			executable='ros2_lora.py',
			name='ros2_lora',
			output='screen'
		)
	])

