from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    mob_description = os.path.join(get_package_share_path('mob_description'))

    gazebo_description = os.path.join(get_package_share_path('ros_gz_sim'))
    # rviz_config_path = os.path.join(get_package_share_path('mob_description'),
    #                                 'rviz', 'urdf_config.rviz')
    
    mob_brignup = os.path.join(get_package_share_path('mob_bringup'))
    mob_launch = os.path.join(mob_description, 'launch', 'mob.launch.py')
    gz_launch = os.path.join(gazebo_description, 'launch', 'gz_sim.launch.py')

    world = os.path.join(mob_brignup, 'worlds', 'world1.sdf')


    rviz2_config_path = os.path.join(mob_brignup,"config","mob_rviz.rviz")

    mob_description_launch = IncludeLaunchDescription( 
        launch_description_source = mob_launch
    )

    mob_gz = IncludeLaunchDescription(
        launch_description_source = gz_launch,
        launch_arguments= {'gz_args': f"{world} -r",
                     }.items()
    )

    parameter_bridge_config = os.path.join(mob_brignup, 'config', 'gz_ros_topic_bridge.yaml')

    mob_create = Node(
        package="ros_gz_sim",
        executable="create",
            arguments=[
                "-topic", "/robot_description",
        ],
        parameters=[{"/use_sim_time":True}]
    )
    

    ros_gz_bridge = Node(
        package=  'ros_gz_bridge',
        executable="parameter_bridge",
        parameters=[
            {"config_file":parameter_bridge_config}]
    )
   
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz2_config_path]
    )

    return LaunchDescription([
        mob_description_launch,
        mob_gz,
        mob_create,
        ros_gz_bridge,
        rviz2_node
    ])