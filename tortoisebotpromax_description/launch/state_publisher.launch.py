import launch
import os
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
import launch_ros
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='tortoisebotpromax_description').find('tortoisebotpromax_description')
    use_sim_time = LaunchConfiguration('use_sim_time')
    default_model_path = os.path.join(pkg_share, 'urdf/tortoisebotpromax.xacro')
    sim_model_path = os.path.join(pkg_share, 'urdf/tortoisebotpromax_sim.xacro')

    # PythonExpression to choose the model based on use_sim_time
    model_path = PythonExpression(["'", default_model_path, "' if not ", use_sim_time, " else '", sim_model_path, "'"])

    xacro_file = Command(['xacro ', model_path])

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': ParameterValue(xacro_file, value_type=str)}]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='true',
                                             description='Flag to enable use_sim_time'),
        robot_state_publisher_node,
        joint_state_publisher_node
    ])

