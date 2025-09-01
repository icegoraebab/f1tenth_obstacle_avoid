from setuptools import setup
from glob import glob  # (런치 파일 포함하려면 필요)
import os # os 모듈 import

package_name = 'f1tenth_obstacle_avoid'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # launch, config, maps 폴더 안의 모든 파일을 설치하도록 설정
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.*')), # maps 폴더 추가
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyeonwoo',
    maintainer_email='ohw2509@gmail.com',
    description='AEB + FTG for F1TENTH',
    license='MIT',
    entry_points={
        'console_scripts': [
            'aeb_node = f1tenth_obstacle_avoid.aeb_node:main',
            'ftg_node = f1tenth_obstacle_avoid.ftg_node:main',
            'scan_preproc_node = f1tenth_obstacle_avoid.scan_preproc_node:main', 
            'odom_to_tf      = f1tenth_obstacle_avoid.odom_to_tf_node:main',
            'odom_tf_broadcaster = f1tenth_obstacle_avoid.odom_tf_broadcaster:main',
        ],
    },
)
