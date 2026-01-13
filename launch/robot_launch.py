# import os
# import launch
# from launch import LaunchDescription
# from ament_index_python.packages import get_package_share_directory
# from webots_ros2_driver.webots_launcher import WebotsLauncher
# from webots_ros2_driver.webots_controller import WebotsController
# def generate_launch_description():
#     package_dir = get_package_share_directory('webots_ros2_husky')
    
#     # Start a Webots simulation instance
#     webots = WebotsLauncher(
#         world=os.path.join(package_dir, 'worlds', 'husky_world.wbt')
#     )

#     # ROS control spawners
#     ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yaml')
    
#     # Create a ROS node interacting with the simulated robot
#     robot_description_path = os.path.join(package_dir, 'resource', 'husky.urdf')
#     robot_driver = WebotsController(
#         robot_name='Husky',
#         parameters=[
#             {'robot_description': robot_description_path},
#             ros2_control_params
#         ]
#     )

#     return LaunchDescription([
#         webots,
#         robot_driver,

#         # The following action will kill all nodes once the Webots simulation has exited
#         launch.actions.RegisterEventHandler(
#             event_handler=launch.event_handlers.OnProcessExit(
#                 target_action=webots,
#                 on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
#         )
#     )
# ])

import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch_ros.actions import Node

def generate_launch_description():
    package_dir = get_package_share_directory('webots_ros2_husky')

    # Start a Webots simulation instance
    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'fire_rescue_world.wbt')
    )

    # Create the robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name=""><link name=""/></robot>'
        }],
    )

    # ROS control spawners
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2control.yaml')
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['joint_state_broadcaster'] + controller_manager_timeout,
    )
    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=['diffdrive_controller'] + controller_manager_timeout,
    )
    ros_control_spawners = [joint_state_broadcaster_spawner, diffdrive_controller_spawner]
    
    mappings = [
        ('/diffdrive_controller/cmd_vel', '/cmd_vel'), 
        ('/diffdrive_controller/odom', '/odom'),
    ]    

    # Create a ROS node interacting with the simulated robot
    robot_description_path = os.path.join(package_dir, 'resource', 'husky.urdf')
    robot_driver = WebotsController(
        robot_name='Husky',
        parameters=[
            {'robot_description': robot_description_path,
             'set_robot_state_publisher': True
             },
            ros2_control_params
        ],
        remappings=mappings
    )

    # Wait for the simulation to be ready to start navigation nodes
    waiting_nodes = WaitForControllerConnection(
        target_driver=robot_driver,
        nodes_to_start=ros_control_spawners
    )    

    return LaunchDescription([
        webots,
        robot_state_publisher,
        robot_driver,
        waiting_nodes,
        # The following action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])