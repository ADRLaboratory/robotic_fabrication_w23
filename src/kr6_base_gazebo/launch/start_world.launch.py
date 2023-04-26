#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('ros_gz_sim')
    pkg_kr6_base_gazebo = get_package_share_directory('kr6_base_gazebo')

    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    description_package_name = "kr6_base_description"
    install_dir = get_package_prefix(description_package_name)

    # Set the path to the WORLD model files. Is to find the models inside the models folder in kr6_base_gazebo package
    gazebo_models_path = os.path.join(pkg_kr6_base_gazebo, 'models')
    # os.environ["GAZEBO_MODEL_PATH"] = gazebo_models_path

    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] =  os.environ['IGN_GAZEBO_RESOURCE_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'IGN_GAZEBO_SYSTEM_PLUGIN_PATH' in os.environ:
        os.environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] = os.environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['IGN_GAZEBO_SYSTEM_PLUGIN_PATH'] = install_dir + '/lib'

    

    print("GAZEBO RESOURCE PATH=="+str(os.environ["IGN_GAZEBO_RESOURCE_PATH"]))
    print("GAZEBO PLUGIN PATH=="+str(os.environ["IGN_GAZEBO_SYSTEM_PLUGIN_PATH"]))

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gz_sim.launch.py'),
        )
    )    

    return LaunchDescription([
        DeclareLaunchArgument(
          'gz_args',
          default_value=[os.path.join(pkg_kr6_base_gazebo, 'worlds', 'kr6_base_empty.world'), ''],
          description='SDF world file'),
        gazebo
    ])