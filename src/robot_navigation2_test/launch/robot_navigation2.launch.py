import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    

    robot_navigation2_dir = get_package_share_directory('robot_navigation2_test')

    nav2_amcl_yaml        = os.path.join(robot_navigation2_dir,'param','nav2_amcl.yaml')
    nav2_behavior_yaml    = os.path.join(robot_navigation2_dir,'param','nav2_behavior.yaml')
    nav2_btnavigator_yaml = os.path.join(robot_navigation2_dir,'param','nav2_btnavigator.yaml')
    nav2_controller_yaml  = os.path.join(robot_navigation2_dir,'param','nav2_controller.yaml')
    nav2_planner_yaml     = os.path.join(robot_navigation2_dir,'param','nav2_planner.yaml')
    nav2_waypointfollower_yaml     = os.path.join(robot_navigation2_dir,'param','nav2_waypointfollower.yaml')

    map_yaml_path = LaunchConfiguration('map',default=os.path.join(robot_navigation2_dir,'maps','gonwei_V3.yaml'))

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
    
    behavior_node =  Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_behavior_yaml])
    
    btnavigator_node =  Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_btnavigator_yaml])
    controller_node =  Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_controller_yaml])
    planner_node =  Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_planner_yaml])
    
    lifecycle_mapper_node =  Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            parameters=[{'use_sim_time': False},
                        {'autostart':True},
                        {'node_names':['map_server',
                                       'amcl']}],
            output='screen')
    
    lifecycle_nav2_node =  Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_nav2',
            parameters=[{'use_sim_time': False},
                        {'autostart':True},
                        {'node_names':['controller_server',
                                       'planner_server',
                                       'behavior_server',
                                       'bt_navigator']}],
            output='screen')

    rviz_node =  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen')
    
    ld = LaunchDescription()
    ld.add_action(map_node)
    ld.add_action(amcl_node)
    ld.add_action(behavior_node)
    ld.add_action(btnavigator_node)
    ld.add_action(controller_node)
    ld.add_action(planner_node)

    ld.add_action(lifecycle_mapper_node)
    ld.add_action(lifecycle_nav2_node)

#     ld.add_action(rviz_node) 

    return ld

