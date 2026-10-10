from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'dobot_camera_calibration'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dobot',
    maintainer_email='dobot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'aruco_pubsub = dobot_camera_calibration.aruco_pubsub:main',
        'hand_eye_calibration = dobot_camera_calibration.hand_eye_calibration:main',
        'feedback = dobot_camera_calibration.feedback:main',
        'ui = dobot_camera_calibration.ui:main',
        ],
    },
)
