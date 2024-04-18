from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    
    #=====================运行节点需要的配置=======================================================================

    usart_port = LaunchConfiguration('usart_port', default='/dev/raspberrypi_base')

    baud_data = LaunchConfiguration('baud_data', default= '115200') 

    robot_frame_id = LaunchConfiguration('robot_frame_id', default='base_link')

    smoother_cmd_vel = LaunchConfiguration('smoother_cmd_vel', default='/cmd_vel')

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
                'smoother_cmd_vel': smoother_cmd_vel,
                'filter_vx_match': filter_vx_match,
                'filter_vth_match': filter_vth_match
            }
        ],
        output='screen')

    teleop_twist_keyboard_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen')


    #===============================================定义启动文件========================================================
    ld = LaunchDescription()
    ld.add_action(teleop_twist_keyboard_node)
    ld.add_action(robot_start_node)

    return ld