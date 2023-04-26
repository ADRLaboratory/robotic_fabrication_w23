import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'kr6_base.urdf.xacro'
    srdf_file = 'kr6_base.srdf'
    #xacro_file = "box_bot.xacro"
    package_description = "kr6_base_moveit_config"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_urdf_path = os.path.join(get_package_share_directory(package_description), "config", urdf_file)
    print("Fetching SRDF ==>")
    robot_srdf_path = os.path.join(get_package_share_directory(package_description), "config", srdf_file)

    # Robot State Publisher

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_urdf_path])}],
        output='screen'
    )

    # MoveIt Group Node

    with open(robot_srdf_path, 'r') as f:
        semantic_content = f.read()
    
    move_group_node = Node(package='moveit_ros_move_group', executable='move_group',
                       name='move_group',
                       parameters=[{
                            'robot_description_semantic': semantic_content,
                            'publish_robot_description_semantic': True
                       }],
                       output='screen'
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package_description), 'rviz', 'urdf_vis.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    # create and return launch description object
    return LaunchDescription(
        [            
            robot_state_publisher_node,
            move_group_node,
            # rviz_node
        ]
    )