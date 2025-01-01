import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'lightweight_human_pose_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'modules'), glob('modules/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'weights'), glob('weights/*')),
        (os.path.join('share', package_name, 'keypoints'), glob('keypoints/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sobits',
    maintainer_email='fumiyaono.choi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # "yolo_node = yolo_ros.yolo_node:main",
            "human_pose_2d = lightweight_human_pose_estimation.human_pose_2d:main",
        ],
    },
)
