from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path
import xacro
def generate_launch_description():

    urdf_path = os.path.join(get_package_share_path('mob_description'),
                             'urdf', 'my_robot.urdf.xacro')
    # rviz_config_path = os.path.join(get_package_share_path('mob_description'),
    #                                 'rviz', 'urdf_config.rviz')
    
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
    


    # robot_description = xacro.process_file(robot_description).toxml()

    # print(robot_description.value[0])
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
    )

    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui"
    # )


    return LaunchDescription([
        robot_state_publisher_node,
        # joint_state_publisher_gui_node,
    ])