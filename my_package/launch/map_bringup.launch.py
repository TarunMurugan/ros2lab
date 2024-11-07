import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():

    ld = LaunchDescription()

    # map file
    map_file_path = os.path.join(
    get_package_share_directory('my_package'),
    'my_map.yaml'
)
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path}])


    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True


    # translation:
    #   x: -1.999277943691657
    #   y: -0.5000000937703502
    #   z: 0.007819793048807812
    # rotation:
    #   x: 1.0905527030369712e-05
    #   y: 0.0033031966781612177
    #   z: 0.0002158330286185427
    #   w: 0.9999945210794301

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    node = Node(package = "tf2_ros", 
                executable = "static_transform_publisher",
                arguments = ["2", "0.5", "0", "0", "0" ,"0", "map", "odom"])

    ld.add_action(node)

    ld.add_action(map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld