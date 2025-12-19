from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import os

def generate_launch_description():

    world_path = os.path.expanduser(
        "~/ros2_ws/src/assessment_world/worlds/assessment_world.sdf"
    )

    robot_description_path = os.path.expanduser(
        "~/ros2_ws/src/my_bot_description/urdf/my_bot.urdf.xacro"
    )

    return LaunchDescription([

        # Launch Gazebo with the assessment world
        ExecuteProcess(
            cmd=["gz", "sim", "-r", world_path],
            output="screen"
        ),

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": os.popen(f"xacro {robot_description_path}").read()
            }],
            output="screen"
        ),

        # Spawn robot in Gazebo
        ExecuteProcess(
            cmd=[
                "ros2", "run", "ros_gz_sim", "create",
                "-name", "my_bot",
                "-topic", "robot_description",
                "-x", "0", "-y", "0", "-z", "0.2"
            ],
            output="screen"
        ),

        # Bridge cmd_vel
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                "/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist"
            ],
            output="screen"
        ),
    ])

