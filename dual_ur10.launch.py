from launch import LaunchDescription
from launch_ros.actions import PushRosNamespace
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ur_launch_file = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'launch',
        'ur_control.launch.py'
    )

    robot1 = GroupAction([
        PushRosNamespace('ur10_1'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_launch_file),
            launch_arguments={
                'robot_ip': '192.168.8.102',
                'ur_type': 'ur10',
                'prefix': 'ur10_1_',
            }.items()
        )
    ])

    robot2 = GroupAction([
        PushRosNamespace('ur10_2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ur_launch_file),
            launch_arguments={
                'robot_ip': '192.168.8.101',
                'ur_type': 'ur10',
                'prefix': 'ur10_2_',
            }.items()
        )
    ])

    return LaunchDescription([robot1, robot2])

