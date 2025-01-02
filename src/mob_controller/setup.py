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
            "runBot = mob_controller.simpleControl:main",
            "objDetect = mob_controller.objDetect:main",
            "obj_transformer = mob_controller.obj_transformer:main",
            "move_action_server = mob_controller.actionsScripts:main",
            "move_action_client = mob_controller.action_client:main",
            "cam_tester = mob_controller.camTest:main",
            "control_robot = mob_controller.controlRobot:main",
            "user_interface = mob_controller.userInterface:main"
        ],
    },
)
