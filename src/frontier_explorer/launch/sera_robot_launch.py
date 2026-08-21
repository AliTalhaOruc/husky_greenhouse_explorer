import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,SetEnvironmentVariable, RegisterEventHandler, TimerAction, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Paket Yolları
    pkg_explorer = get_package_share_directory('frontier_explorer')
    pkg_husky_gazebo = get_package_share_directory('husky_gazebo')
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    husky_gazebo_path = get_package_share_directory('husky_gazebo')
    models_path = os.path.join(husky_gazebo_path, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += ':' + models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = models_path

    pkg_rviz = get_package_share_directory('rviz2')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    current_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_model_path = f"{models_path}:{current_model_path}" if current_model_path else models_path
    # Kaydettiğin Dünya Dosyasının Yolu:
    world_path = os.path.join(pkg_husky_gazebo, 'worlds', 'sera_world.world')

    # Parametre Dosyaları (frontier_explorer/config içinde)
    imu_config = os.path.join(pkg_explorer, 'config', 'imu_filter.yaml')
    ekf_config = os.path.join(pkg_explorer, 'config', 'ekf.yaml')
    slam_params = os.path.join(pkg_explorer, 'config', 'husky_slam_params.yaml')
    nav2_params = os.path.join(pkg_explorer, 'config', 'husky_nav.yaml')

    # --- 1. ADIM: GAZEBO SİMÜLASYONU VE ROBOT SPAWN ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_husky_gazebo, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world_path': world_path}.items()
    )

    # --- 2. ADIM: IMU & EKF DÜĞÜMLERİ ---
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        parameters=[imu_config, {'use_sim_time': use_sim_time}],
        remappings=[('imu/data_raw', '/imu/data_raw'), ('imu/data', '/imu/data')]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}]
    )

    # --- 3. ADIM: SLAM ---
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params, {'use_sim_time': use_sim_time}]
    )

    # --- 4. ADIM: NAV2 ---
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'use_composition': 'False',
            'slam': 'True'
        }.items()
    )

    # --- 5. ADIM: EXPLORER (Otonom Taramayı Başlatan Düğüm) ---
    explorer = Node(
        package='frontier_explorer',
        executable='explorer',
        name='explorer_node',
        parameters=[{'use_sim_time': True}]
    )

    # --- ZAMANLAMA VE TETİKLEME ZİNCİRİ ---
    
    # Gazebo açıldıktan 6 saniye sonra IMU ve EKF'yi başlat
    start_nodes_after_gazebo = TimerAction(
        period=6.0,
        actions=[imu_filter_node, ekf_node]
    )

    # EKF başladıktan sonra SLAM'i başlat
    start_slam_on_ekf = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=ekf_node,
            on_start=[slam]
        )
    )

    # SLAM başladıktan sonra Nav2'yi başlat
    start_nav2_on_slam = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=slam,
            on_start=[nav2]
        )
    )

    # Nav2 başladıktan 8 saniye sonra Explorer (Otonom Keşif) başlatılsın
    start_explorer_with_delay = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=slam,
            on_start=[
                TimerAction(
                    period=8.0,
                    actions=[explorer],
                )
            ]
        )
    )

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', new_model_path),
        gazebo_launch,
        start_nodes_after_gazebo,
        start_slam_on_ekf,
        start_nav2_on_slam,
        start_explorer_with_delay
    ])
