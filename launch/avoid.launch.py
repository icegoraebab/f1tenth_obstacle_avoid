from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('f1tenth_obstacle_avoid')
    # 설치된 config 폴더( share/<pkg>/config )의 ftg.yaml 사용
    ftg_cfg = os.path.join(pkg, 'config', 'ftg.yaml')

    return LaunchDescription([
        Node(
            package='f1tenth_obstacle_avoid',
            executable='ftg_node',     # setup.py의 console_scripts 이름
            parameters=[ftg_cfg],
            output='screen'
        ),
        Node(
            package='f1tenth_obstacle_avoid',
            executable='aeb_node',
            # 필요하면 aeb.yaml 만들어서 parameters=[os.path.join(pkg, 'config', 'aeb.yaml')]
            parameters=[{'front_angle_deg': 30.0, 'stop_dist': 0.7}],
            output='screen'
        ),
    ])
