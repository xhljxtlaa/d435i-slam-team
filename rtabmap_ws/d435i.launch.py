from datetime import datetime
from pathlib import Path
from tempfile import mkdtemp

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def package_launch(package, filename):
    return PythonLaunchDescriptionSource(
        str(Path(get_package_share_directory(package)) / 'launch' / filename)
    )


def launch_mapping(context):
    try:
        mapping_launch = package_launch('rtabmap_launch', 'rtabmap.launch.py')
    except PackageNotFoundError as error:
        raise RuntimeError(
            '缺少 RTAB-Map。请在本机终端运行 sudo apt install ros-humble-rtabmap-launch'
        ) from error

    camera_namespace = 'camera'
    camera_name = 'camera'
    camera_topic = f'/{camera_namespace}/{camera_name}'
    actions = []
    if LaunchConfiguration('start_camera').perform(context) == 'true':
        actions.append(Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace=camera_namespace,
            name=camera_name,
            output='screen',
            parameters=[{
                'enable_color': True,
                'enable_depth': True,
                'enable_infra1': False,
                'enable_infra2': False,
                'enable_accel': False,
                'enable_gyro': False,
                'rgb_camera.color_profile': '640x480x15',
                'depth_module.depth_profile': '640x480x15',
                'depth_module.emitter_enabled': 1,
                'align_depth.enable': True,
                'enable_sync': True,
            }],
        ))

    output = Path(__file__).resolve().parent / 'output'
    output.mkdir(exist_ok=True)
    run = Path(mkdtemp(prefix=datetime.now().strftime('run_%Y%m%d_%H%M%S_'), dir=output))
    database = str(run / 'rtabmap.db')
    return actions + [
        LogInfo(msg=f'RTAB-Map 数据库路径：{database}'),
        IncludeLaunchDescription(mapping_launch, launch_arguments={
            'database_path': database,
            'frame_id': f'{camera_name}_link',
            'rgb_topic': f'{camera_topic}/color/image_raw',
            'depth_topic': f'{camera_topic}/aligned_depth_to_color/image_raw',
            'camera_info_topic': f'{camera_topic}/color/camera_info',
            # Pair RGB and depth once. The odometry and SLAM nodes then share the same frames.
            'rgbd_sync': 'true',
            'approx_rgbd_sync': 'true',
            'approx_sync_max_interval': '0.02',
            'qos': '1',
            'odom_args': '--Odom/ResetCountdown 0',
            'rtabmap_viz': LaunchConfiguration('show_viewer'),
        }.items()),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rtabmap_rviz',
            arguments=['-d', str(Path(__file__).with_name('rtabmap.rviz'))],
            output='screen',
            condition=IfCondition(LaunchConfiguration('show_rviz')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('start_camera', default_value='true', choices=['true', 'false'],
                              description='Start the RealSense D435i driver.'),
        DeclareLaunchArgument('show_viewer', default_value='true', choices=['true', 'false'],
                              description='Start the RTAB-Map desktop viewer.'),
        DeclareLaunchArgument('show_rviz', default_value=LaunchConfiguration('show_viewer'),
                              choices=['true', 'false'],
                              description='Start RViz with RGB, depth, trajectory and cloud.'),
        OpaqueFunction(function=launch_mapping),
    ])
