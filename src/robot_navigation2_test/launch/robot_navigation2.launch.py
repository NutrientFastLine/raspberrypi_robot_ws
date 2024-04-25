import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    

    robot_navigation2_dir = get_package_share_directory('robot_navigation2_test')

    nav2_amcl_yaml = os.path.join(robot_navigation2_dir,'param','robot_nav2_amcl.yaml')
    map_yaml_path = LaunchConfiguration('map',default=os.path.join(robot_navigation2_dir,'maps','test1_map.yaml'))
    rviz_config_dir = os.path.join(robot_navigation2_dir,'rviz2','robot_nav2_test.rviz')

    amcl_node =  Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[nav2_amcl_yaml],
            output='screen')
    
    map_node =  Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': False}, 
                        {'yaml_filename':map_yaml_path} 
                       ])
    
    rviz_node =  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen')
    
    lifecycle_node =  Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            parameters=[{'use_sim_time': False},
                        {'autostart':True},
                        {'node_names':['map_server',
                                       'amcl']}],
        #     parameters=[{'use_sim_time': False},
        #                 {'autostart':True},
        #                 {'node_names':['map_server']}],
            output='screen')
    
    ld = LaunchDescription()
    ld.add_action(map_node)
    ld.add_action(amcl_node)
#     ld.add_action(rviz_node) 
    ld.add_action(lifecycle_node)


    return ld

