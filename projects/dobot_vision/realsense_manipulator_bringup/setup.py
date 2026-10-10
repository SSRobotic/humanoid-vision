from setuptools import find_packages, setup

package_name = 'realsense_manipulator_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dobot,FTX',
    maintainer_email='dobot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'realsense_pubsub_listener = realsense_manipulator_bringup.realsense_pubsub_listener:main',
        'realsense_pubsub_talker = realsense_manipulator_bringup.realsense_pubsub_talker:main',
        'realsense_camera_bringup = realsense_manipulator_bringup.realsense_camera_bringup:main',
        ],
    },
)
