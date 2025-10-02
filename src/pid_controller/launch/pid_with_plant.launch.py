from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # create a node from the pid node woth intialised params
        Node(
            package="pid_controller",
            executable="pid_node",
            name="pid_node",
            output="screen",
            parameters=[{
                "kp": 1.0,
                "ki": 0.1,
                "kd": 0.05,
            }]
        ),

        # create an instance of the plant simualtiuon node
        Node(
            package="pid_controller",
            executable="plant_node",
            name="plant_node",
            output="screen"
        ),
    ])
