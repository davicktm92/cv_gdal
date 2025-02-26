import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml


def generate_launch_description():
    
    #import of map_name based on yaml of previous algorithm
    with open(os.path.join(get_package_share_directory('cv_gdal'), 'maps', 'map1.yaml'), 'r') as file:
        map_conf = yaml.safe_load(file)
        zone=map_conf['zone_name']

    world_file = PathJoinSubstitution(
        [FindPackageShare("cv_gdal"),
        "worlds",
        str(zone)+".world"],
    )

    gazebo_launch = PathJoinSubstitution(
        [FindPackageShare("husky_navigation"),
        "launch",
        "multi_gps_navigation.launch.py"],
    )

    map_server_config_path_cabrera = os.path.join(
    get_package_share_directory('cv_gdal'),
    'maps',
    'map1.yaml'
    )

    # gazebo_launch = PathJoinSubstitution(
    #     [FindPackageShare("husky_gazebo"),
    #     "launch",
    #     "multirobot.launch.py"],
    # )

    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gazebo_launch]),
        launch_arguments={'world_path': world_file,
                          "map_path": map_server_config_path_cabrera}.items(),
    )

    map_tf=Node(
        package='cv_gdal',
        executable='map_tf.py',
        name='map_tf',
        output='screen',
    )

    path_follower = Node(
        package='cv_gdal',
        executable='path_follower.py',
        name='path_follower',
        output='screen',
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_sim)
    ld.add_action(path_follower)
    ld.add_action(map_tf)

    return ld