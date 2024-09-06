from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  wiferion_ros2_node = Node(
    package='wiferion_ros2',
    executable='wiferion_ros2',
    parameters=[
      {
        'port': 'can0',
        'baud': 250000
      }
    ]
  )

  ld = LaunchDescription()
  ld.add_action(wiferion_ros2_node)

  return ld