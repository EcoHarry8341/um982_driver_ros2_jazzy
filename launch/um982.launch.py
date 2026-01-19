from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nmea_navsat_driver',
            executable='nmea_serial_driver',
            name='um982_driver',
            output='screen',
            parameters=[{
                'port': '/dev/rtk',      # 咱们之前绑定的别名
                'baud': 115200,          # 你的设备波特率
                'frame_id': 'gps_link',  # 坐标系ID
                'time_ref_source': 'gps',
                'useRMC': False
            }]
        )
    ])