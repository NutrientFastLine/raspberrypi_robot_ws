import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    #=============================1.定位到包的地址=============================================================
    robot_navigation2_dir = get_package_share_directory('robot_navigation2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    #=============================2.声明参数，获取配置文件路径===================================================
    # use_sim_time 这里要设置成true,因为gazebo是仿真环境，其时间是通过/clock话题获取，而不是系统时间
    use_sim_time = LaunchConfiguration('use_sim_time', default='false') 
    map_yaml_path = LaunchConfiguration('map',default=os.path.join(robot_navigation2_dir,'maps','test1_map.yaml'))
    nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(robot_navigation2_dir,'param','robot_nav2.yaml'))
    rviz_config_dir = os.path.join(robot_navigation2_dir,'rviz2','robot_nav2.rviz')

    #=============================3.声明启动launch文件，传入：地图路径、是否使用仿真时间以及nav2参数文件==============
    nav2_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
        )
    rviz_node =  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    

    #=========定义启动文件========================================================
    ld = LaunchDescription()
    ld.add_action(nav2_bringup_launch)
#     ld.add_action(rviz_node)

#     robot_navigation2_dir = get_package_share_directory('robot_navigation2')

#     nav2_amcl_yaml = os.path.join(robot_navigation2_dir,'param','robot_nav2_amcl.yaml')
#     map_yaml_path = LaunchConfiguration('map',default=os.path.join(robot_navigation2_dir,'maps','test1_map.yaml'))
#     rviz_config_dir = os.path.join(robot_navigation2_dir,'rviz2','robot_nav2_test.rviz')

#     amcl_node =  Node(
#             package='nav2_amcl',
#             executable='amcl',
#             name='amcl',
#             parameters=[nav2_amcl_yaml],
#             output='screen')
    
#     map_node =  Node(
#             package='nav2_map_server',
#             executable='map_server',
#             name='map_server',
#             output='screen',
#             parameters=[{'use_sim_time': False}, 
#                         {'yaml_filename':map_yaml_path} 
#                        ])
    
#     rviz_node =  Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             arguments=['-d', rviz_config_dir],
#             output='screen')
    
#     lifecycle_node =  Node(
#             package='nav2_lifecycle_manager',
#             executable='lifecycle_manager',
#             name='lifecycle_manager_mapper',
#             parameters=[{'use_sim_time': False},
#                         {'autostart':True},
#                         {'node_names':['map_server',
#                                        'amcl']}],
#         #     parameters=[{'use_sim_time': False},
#         #                 {'autostart':True},
#         #                 {'node_names':['map_server']}],
#             output='screen')
    
#     ld = LaunchDescription()
#     ld.add_action(map_node)
#     ld.add_action(amcl_node)
# #     ld.add_action(rviz_node) 
#     ld.add_action(lifecycle_node)


    return ld

