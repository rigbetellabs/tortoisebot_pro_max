import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='tortoisebotpromax_gazebo').find('tortoisebotpromax_gazebo')
    world_path=os.path.join(pkg_share, 'worlds/small_house.world'),
    use_sim_time = LaunchConfiguration('use_sim_time')

    return launch.LaunchDescription([
        launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 
                                            'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',world_path], 
                                           output='screen'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                description='Flag to enable use_sim_time'),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'tortoisebotpromax', '-topic', 'robot_description'],
            parameters= [{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
    
