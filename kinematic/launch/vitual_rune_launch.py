import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


# 获得config的路径
config_file = os.path.join(
    get_package_share_directory('kinematic'),
    'config',
    'rune_param.yaml'
)


workspace = os.path.split(get_package_prefix('kinematic'))[0]
workspace = os.path.split(workspace)[0]

# 获得config的路径
config = os.path.join(
    workspace,
    'src',
    'config'
)
car_id = os.getenv('CAR_ID', '3')

# config下的文件路径
g_camera_param_path = os.path.join(config, 'camera_param'+car_id+'.xml')
g_mv_camera_config_path = os.path.join(
    config, 'mv_camera_config'+car_id+'.yaml')
g_tune_param_path = os.path.join(config, 'tune_param'+car_id+'.yaml')
g_other_param_path = os.path.join(config, 'other_param.xml')


def generate_launch_description():
    # 声明为launch参数
    declare_g_camera_param_path = DeclareLaunchArgument(
        'g_camera_param_path',
        default_value=g_camera_param_path,
        description='')
    declare_g_mv_camera_config_path = DeclareLaunchArgument(
        'g_mv_camera_config_path',
        default_value=g_mv_camera_config_path,
        description=''
    )
    declare_g_tune_param_path = DeclareLaunchArgument(
        'g_tune_param_path',
        default_value=g_tune_param_path,
        description=''
    )
    declare_g_other_param_path = DeclareLaunchArgument(
        'g_other_param_path',
        default_value=g_other_param_path,
        description=''
    )
    declare_car_id = DeclareLaunchArgument(
        'car_id',
        default_value=car_id,
        description=''
    )

    car_id_arg = LaunchConfiguration('car_id')
    g_camera_param_path_arg = LaunchConfiguration('g_camera_param_path')
    g_mv_camera_config_path_arg = LaunchConfiguration(
        'g_mv_camera_config_path')
    g_tune_param_path_arg = LaunchConfiguration('g_tune_param_path')
    g_other_param_path_arg = LaunchConfiguration('g_other_param_path')
    path_para = {"g_camera_param_path": g_camera_param_path_arg,
                 "g_mv_camera_config_path": g_mv_camera_config_path_arg,
                 "g_tune_param_path": g_tune_param_path_arg,
                 'g_other_param_path': g_other_param_path_arg,
                 'car_id': car_id_arg}

    declare_virtual_rune_log_level_arg = DeclareLaunchArgument(
        "virtual_rune_log-level",
        default_value=["INFO"],
        description="Logging level",
    )
    virtual_rune_log_level = LaunchConfiguration("virtual_rune_log-level")

    return launch.LaunchDescription([
        declare_g_camera_param_path,
        declare_g_mv_camera_config_path,
        declare_g_tune_param_path,
        declare_g_other_param_path,
        declare_car_id,
        declare_virtual_rune_log_level_arg,
        launch_ros.actions.Node(
            package='kinematic',
            executable='virtual_rune',
            name='virtual_rune',
            output='both',
            arguments=[
                    "--ros-args",
                    "--log-level",
                    ["virtual_rune:=", virtual_rune_log_level],
            ],
            parameters=[config_file, path_para])
    ]
    )
