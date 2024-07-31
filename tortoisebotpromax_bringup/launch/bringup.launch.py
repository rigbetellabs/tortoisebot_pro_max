import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  state_publisher_launch_dir=os.path.join(get_package_share_directory('tortoisebotpromax_description'), 'launch')
  firmware_dir = os.path.join(get_package_share_directory('tortoisebotpromax_firmware'), 'launch')
  X4_Pro_params_dir=os.path.join(get_package_share_directory('tortoisebotpromax_firmware'),'config','X4_Pro_params.yaml')

  joy = LaunchConfiguration('joy')
  state_publisher_launch_cmd=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(state_publisher_launch_dir, 'state_publisher.launch.py')),)

  ydlidar_launch_cmd=LifecycleNode(package='ydlidar_ros2_driver',
                                executable='ydlidar_ros2_driver_node',
                                name='ydlidar_ros2_driver_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[X4_Pro_params_dir],
                                namespace='/',
                                )

  microros_node=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(firmware_dir, 'micro_ros.launch.py')),
        )

  realsense_launch=IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(firmware_dir, 'realsense_d435i.launch.py')),
        )
  
  hubble_scripts_launch=IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(firmware_dir, 'hubble_scripts.launch.py')),
        )
  auto_joy_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(firmware_dir, 'auto_joy_teleop.launch.py')),
        condition=IfCondition(joy),
    )

  return LaunchDescription([

    SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    DeclareLaunchArgument(name='joy', default_value='True',
                                              description='To enable joystick control'),

    state_publisher_launch_cmd,
    ydlidar_launch_cmd,
    realsense_launch, 
    microros_node, 
    hubble_scripts_launch,
    auto_joy_cmd

  ]
)
