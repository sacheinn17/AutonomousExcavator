from setuptools import find_packages, setup

package_name = 'mob_controller'

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
    maintainer='sac',
    maintainer_email='sac@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robo_controller = mob_controller.robo_controller:main",
            "show_video = mob_controller.show_video:main",
            "mob_teleop_twist_keyboard = mob_controller.mob_teleop_twist_keyboard:main",
            "cam_feeder = mob_controller.video_feeder:main",
            "control_robot = mob_controller.controlRobot:main",
        ],
    },
)
