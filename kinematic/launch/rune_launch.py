import launch
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# 获取config路径
config_file = os.path.join(
    get_package_share_directory('kinematic'),
    'config',
    'rune_param.yaml'
)

# 环境变量处理
car_id = os.getenv('CAR_ID', '3')
workspace = os.path.split(get_package_prefix('kinematic'))[0]
workspace = os.path.split(workspace)[0]

# 动态路径生成
path_configs = {
    'g_camera_param_path': os.path.join(workspace, 'src/config', f'camera_param{car_id}.xml'),
    'g_mv_camera_config_path': os.path.join(workspace, 'src/config', f'mv_camera_config{car_id}.yaml'),
    'g_tune_param_path': os.path.join(workspace, 'src/config', f'tune_param{car_id}.yaml'),
    'g_other_param_path': os.path.join(workspace, 'src/config', 'other_param.xml')
}


def generate_launch_description():
    # 声明launch参数
    declare_arguments = [
        DeclareLaunchArgument(name, default_value=path, description='')
        for name, path in path_configs.items()
    ] + [
        DeclareLaunchArgument(
            'car_id', default_value=car_id, description='Vehicle ID'),
        DeclareLaunchArgument('log_level', default_value='debug',
                              description='Logging level (debug, info, warn, error)')
    ]
    car_id_arg = LaunchConfiguration('car_id')

    # 创建共享参数字典
    shared_params = {
        **path_configs,
        'car_id': car_id_arg
    }

    # 组件容器配置
    container = ComposableNodeContainer(
        name='kinematic_container',
        namespace='rune',
        package='rclcpp_components',
        executable='component_container_mt',  # 多线程容器
        parameters=[config_file, shared_params],
        output='both',
        composable_node_descriptions=[
            # 预处理器组件
            ComposableNode(
                package='kinematic',
                plugin='rune::RunePreprocessor',
                name='rune_preprocessor',
                parameters=[config_file, shared_params],
                extra_arguments=[{
                    'use_intra_process_comms': True,
                }]
            ),
            # 建模组件
            ComposableNode(
                package='kinematic',
                plugin='rune::RuneModeling',
                name='rune_modeling',
                parameters=[config_file, shared_params],
                extra_arguments=[{
                    'use_intra_process_comms': True,
                }]
            ),
            # 火力控制组件
            ComposableNode(
                package='kinematic',
                plugin='rune::RuneFireController',
                name='rune_fire_controller',
                parameters=[config_file, shared_params],
                extra_arguments=[{
                    'use_intra_process_comms': True,
                }]
            ),
        ]
    )
    # set_rune_preprocessor_log_level = ExecuteProcess(
    #     cmd=['ros2', 'service', 'call', '/rune_preprocessor/set_logger_level',
    #          'rcl_interfaces/srv/SetLoggerLevel',
    #          "logger:='rune_preprocessor' level:='info'"],
    #     shell=True
    # )
    # set_rune_modeling_log_level = ExecuteProcess(
    #     cmd=['ros2', 'service', 'call', '/rune_modeling/set_logger_level',
    #          'rcl_interfaces/srv/SetLoggerLevel',
    #          "logger:='rune_modeling' level:='debug'"],
    #     shell=True
    # )
    # set_rune_fire_controller_log_level = ExecuteProcess(
    #     cmd=['ros2', 'service', 'call', '/rune_fire_controller/set_logger_level',
    #          'rcl_interfaces/srv/SetLoggerLevel',
    #          "logger:='rune_fire_controller' level:='info'"],
    #     shell=True
    # )

    return launch.LaunchDescription([
        *declare_arguments,
        container,
        # TimerAction(period=2.0, actions=[
        #             set_rune_preprocessor_log_level, set_rune_modeling_log_level, set_rune_fire_controller_log_level]),
        LogInfo(msg=f"Kinematic System Started with CAR_ID={car_id}"),
        LogInfo(msg=f"Intra-process communication: Enabled")
    ])
