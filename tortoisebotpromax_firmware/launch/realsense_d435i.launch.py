import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    realsense_launch_dir = os.path.join(get_package_share_directory('realsense2_camera'), 'launch')
    
    # Launch RealSense camera
    camera_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_launch_dir, 'rs_launch.py')
        ),
        launch_arguments={
            'pointcloud.enable': 'True',
            'depth_module.auto_exposure_limit': '1',
            'depth_module.auto_gain_limit': '1',
            'depth_module.auto_exposure_limit_toggle': 'False',
            'depth_module.auto_gain_limit_toggle': 'False',
            'rgb_camera.power_line_frequency': '1',
            'enable_gyro': 'True',
            'enable_accel': 'True',
            'unit_imu_method': "2",
            'rgb_camera.profile': '640,360,6',
        }.items()
    )

    
    return LaunchDescription([
        camera_launch_cmd,
    ])
