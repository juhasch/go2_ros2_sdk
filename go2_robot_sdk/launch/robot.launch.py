# Copyright (c) 2024, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    with_rviz2 = LaunchConfiguration('rviz2', default='true')
    with_nav2 = LaunchConfiguration('nav2', default='true')
    with_slam = LaunchConfiguration('slam', default='true')
    with_joystick = LaunchConfiguration('joystick', default='true')
    with_teleop = LaunchConfiguration('teleop', default='true') 

    robot_token = os.getenv('ROBOT_TOKEN', '') # how does this work for multiple robots?
    robot_ip = os.getenv('ROBOT_IP', '')
    robot_ip_lst = robot_ip.replace(" ", "").split(",")

    x =PythonLaunchDescriptionSource([
                    os.path.join(get_package_share_directory(
                        'nav2_bringup'), 'launch', 'navigation_launch.py')
                ])

    # these are debug only
    map_name = os.getenv('MAP_NAME', '3d_map')
    save_map = os.getenv('MAP_SAVE', 'true')

    conn_type = os.getenv('CONN_TYPE', 'webrtc')

    conn_mode = "single" if len(robot_ip_lst) == 1 and conn_type != "cyclonedds" else "multi"

    if conn_mode == 'single':
        rviz_config = "single_robot_conf.rviz"
    else:
        rviz_config = "multi_robot_conf.rviz"

    if conn_type == 'cyclonedds':
        rviz_config = "cyclonedds_config.rviz"

    urdf_file_name = 'go2.urdf'
    urdf = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        "urdf",
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_desc_modified_lst = []

    for i in range(len(robot_ip_lst)):
        robot_desc_modified_lst.append(robot_desc.format(robot_num=f"robot{i}"))

    urdf_launch_nodes = []

    joy_params = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config', 'joystick.yaml'
    )
    
    teleop_params = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config', 'teleop_twist_joy.yaml'
    )                  
    default_config_topics = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config', 'twist_mux.yaml')

    slam_toolbox_config = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config',
        'mapper_params_online_async.yaml'
    )

    nav2_config = os.path.join(
        get_package_share_directory('go2_robot_sdk'),
        'config',
        'nav2_params.yaml'
    )

    if conn_mode == 'single':
        urdf_file_name = 'go2.urdf'
        urdf = os.path.join(
            get_package_share_directory('go2_robot_sdk'),
            "urdf",
            urdf_file_name)
        with open(urdf, 'r') as infp:
            robot_desc = infp.read()

        urdf_launch_nodes.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': use_sim_time,
                             'robot_description': robot_desc}],
                arguments=[urdf]
            ),
        )
        urdf_launch_nodes.append(
            Node(
                package='pointcloud_to_laserscan',
                executable='pointcloud_to_laserscan_node',
                name='pointcloud_to_laserscan',
                remappings=[
                    ('cloud_in', 'point_cloud2'),
                    ('scan', 'scan'),
                ],
                parameters=[{
                    'target_frame': 'base_link',
#                    'transform_tolerance': 0.01,
#                    'min_height': -0.1,
                    'max_height': 0.5,
#                    'angle_min': -3.14,  # -M_PI/2
#                    'angle_max': 3.14,  # M_PI/2
#                    'angle_increment': 0.0087,  # M_PI/360.0
#                    'scan_time': 0.25,
#                    'range_min': 0.10,
#                    'range_max': 20.0,
#                    'use_inf': True,
#                    'inf_epsilon': 1.0
                }],
                output='screen',
            ),
        )
        urdf_launch_nodes.append(
            Node(
                package='go2_perception', executable='cloud_accumulation',
                remappings=[
                    ('cloud', '/trans_cloud')
                    ],
                name='cloud_accumulation'
            ),
        )
    else:
        for i in range(len(robot_ip_lst)):
            urdf_launch_nodes.append(
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    namespace=f"robot{i}",
                    parameters=[{'use_sim_time': use_sim_time,
                                 'robot_description': robot_desc_modified_lst[i]}],
                    arguments=[urdf]
                ),
            )
            urdf_launch_nodes.append(
                Node(
                    package='go2_perception',
                    executable='pointcloud_to_laserscan_node',
                    name='pointcloud_to_laserscan',
                    remappings=[
                        ('cloud_in', f'robot{i}/point_cloud2'),
                        ('scan', f'robot{i}/scan'),
                    ],
                    parameters=[{
                        'target_frame': f'robot{i}/base_link',
                        'max_height': 0.1
                    }],
                    output='screen',
                ),
            )

    return LaunchDescription([
        *urdf_launch_nodes,
        Node(
            package='go2_robot_sdk',
            executable='go2_driver_node',
            parameters=[{'robot_ip': robot_ip, 'token': robot_token, "conn_type": conn_type}],
            remappings=[('cmd_vel', 'cmd_vel_robot')],
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            parameters=[{'robot_ip_lst': robot_ip_lst, 'map_name': map_name, 'map_save': save_map}],
        ),
        Node(
            package='go2_perception', executable='cloud_accumulation',
            remappings=[
                ('cloud', '/trans_cloud')
                ],
            name='cloud_accumulation'
        ),
        Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            condition=IfCondition(with_rviz2),
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('go2_robot_sdk'), 'config', rviz_config)]
        ),
        Node(
            package='joy',
            executable='joy_node',
            condition=IfCondition(with_joystick),
            parameters=[joy_params]
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            condition=IfCondition(with_joystick),
            parameters=[joy_params],
            remappings=[('cmd_vel', 'cmd_vel_joy')]
        ),
        Node(
            package='twist_mux',
            executable='twist_mux',
            output='screen',
            condition=IfCondition(with_teleop),
            remappings=[('cmd_vel_out', 'cmd_vel_robot')],
            parameters=[
                {'use_sim_time': use_sim_time},
                default_config_topics,
                {'use_stamped': False}
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory(
                    'slam_toolbox'), 'launch', 'online_async_launch.py')
            ]),
            condition=IfCondition(with_slam),
            launch_arguments={
                'slam_params_file': slam_toolbox_config,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        # Nav2 nodes launched individually
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            condition=IfCondition(with_nav2),
            parameters=[nav2_config],
            remappings=[('cmd_vel', 'nav_cmd_vel')],
            output='screen'
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            condition=IfCondition(with_nav2),
            parameters=[nav2_config],
            output='screen'
        ),
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            condition=IfCondition(with_nav2),
            parameters=[nav2_config],
            remappings=[('cmd_vel', 'nav_cmd_vel')],
            output='screen'
        ),
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            condition=IfCondition(with_nav2),
            parameters=[nav2_config],
            output='screen'
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            condition=IfCondition(with_nav2),
            parameters=[{'autostart': True,
                        'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']}],
            output='screen'
        ),


    ])
