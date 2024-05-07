import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    #==========启动robot_desscription URDF文件========================================================
    
    desscription_pkg_share = FindPackageShare(package='robot_description').find('robot_description') 
    desscription_urdf_model_path = os.path.join(desscription_pkg_share, f'urdf/{"robot_base.urdf"}')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[desscription_urdf_model_path]
        )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[desscription_urdf_model_path]
        )    

    #=====================运行底盘节点需要的配置==========================================================

    usart_port = LaunchConfiguration('usart_port', default='/dev/raspberrypi_base')
    baud_data = LaunchConfiguration('baud_data', default= '115200') 
    robot_frame_id = LaunchConfiguration('robot_frame_id', default='base_link')  
    odom_child_id = LaunchConfiguration('odom_child_id', default='base_link')
    smoother_cmd_vel = LaunchConfiguration('smoother_cmd_vel', default='/cmd_vel')
    odom_pub_topic = LaunchConfiguration('odom_pub_topic', default='/odom_diff')
    filter_vx_match = LaunchConfiguration('filter_vx_match', default='1.0')
    filter_vth_match = LaunchConfiguration('filter_vth_match', default='1.0')

    robot_start_node = Node(
        package='robot_start',
        executable='robot_start',
        name='robot_start',
        parameters=[
            {
                'usart_port': usart_port,
                'baud_data': baud_data,
                'robot_frame_id': robot_frame_id,
                'odom_child_id' : odom_child_id,
                'smoother_cmd_vel': smoother_cmd_vel,
                'odom_pub_topic' : odom_pub_topic,
                'filter_vx_match': filter_vx_match,
                'filter_vth_match': filter_vth_match,
            }
        ],
        output='screen')
    
    #=====================运行雷达节点需要的配置==========================================================

    channel_type =  LaunchConfiguration('channel_type', default='serial')   
    serial_port = LaunchConfiguration('serial_port', default='/dev/raspberrypi_sllidar')   
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser_link')   
    inverted = LaunchConfiguration('inverted', default='false')  
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')   
    # scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity')

    sllidar_ros2_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[
            {
                'channel_type': channel_type,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': inverted,
                'angle_compensate': angle_compensate
            }
        ],
        output='screen')

    #==========启动robot_localization 融合odom和imu数据========================================================
    
    robot_start_pkg = FindPackageShare(package='robot_start').find('robot_start') 

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(robot_start_pkg, 'params', 'ekf.yaml')],
    )   
    
    #===============================================定义启动文件========================================================

    ld = LaunchDescription()
    
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)

    ld.add_action(robot_start_node)
    ld.add_action(sllidar_ros2_node)

    ld.add_action(ekf_node)

    return ld
