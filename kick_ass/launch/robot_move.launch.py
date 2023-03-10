import os
from launch import LaunchDescription
from launch_ros.actions import Node
 
 
def generate_launch_description():
 
    f_node = Node(
        package="kick_ass",
        executable="estimate",
    )

    s_node = Node(
        package="kick_ass",
        executable="control",
    )
    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(f_node)
    ld.add_action(s_node)

    return ld