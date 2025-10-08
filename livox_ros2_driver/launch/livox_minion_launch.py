import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch

################### user configure parameters for ros2 start ###################
xfer_format   = 0    # 0-Pointcloud2(PointXYZRTL), 1-customized pointcloud format
multi_topic   = 1    # 0-All LiDARs share the same topic, 1-One LiDAR one topic
data_src      = 0    # 0-lidar,1-hub
publish_freq  = 10.0 # freqency of publish,1.0,2.0,5.0,10.0,etc
output_type   = 0
lvx_file_path = '/home/livox/livox_test.lvx'
cmdline_bd_code = 'livox0000000001'

cur_path = os.path.split(os.path.realpath(__file__))[0] + '/'
cur_config_path = cur_path + '../config'
rviz_config_path = os.path.join(cur_config_path, 'livox_lidar.rviz')
user_config_path = os.path.join(cur_config_path, 'livox_lidar_config.json')
################### user configure parameters for ros2 end #####################

# Define unique frame IDs for each LiDAR
lidar_frame_map = {
    "3WEDJAR00100221": "livox_center",
    "1HDDGC500102611": "livox_port",
    "1HDDH3200106291": "livox_sb"
}

def generate_launch_description():
    launch_entities = []
    
    # Create nodes for each LiDAR
    for broadcast_code, frame_id in lidar_frame_map.items():
        node = Node(
            package='livox_ros2_driver',
            executable='livox_ros2_driver_node',
            name=f'livox_{broadcast_code}_publisher',
            output='screen',
            parameters=[
                {"xfer_format": xfer_format},
                {"multi_topic": multi_topic},
                {"data_src": data_src},
                {"publish_freq": publish_freq},
                {"output_data_type": output_type},
                {"frame_id": frame_id},
                {"lvx_file_path": lvx_file_path},
                {"user_config_path": user_config_path},
                {"cmdline_input_bd_code": broadcast_code}  # Changed to use actual broadcast code
            ]
        )
        launch_entities.append(node)
    
    # Return LaunchDescription with all entities
    return LaunchDescription(launch_entities)
