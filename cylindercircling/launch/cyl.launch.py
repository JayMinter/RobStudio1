# from launch import LaunchDescription
# from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.actions import Node
# import os

# def generate_launch_description():
#     # Path to your map
#     map_file = os.path.expanduser('~/ros2_ws/RTAB_MAPS/farmWith3CropRows/Sprint4Map.yaml')
#     # Path to your custom RViz config file
#     rviz_config_path = os.path.expanduser('~/ros2_ws/custom_config.rviz')

#     config_dir = os.path.join(os.path.expanduser('~/ros2_ws/src/cylindercircling/src/config'))

#     return LaunchDescription([

#         # Set the TurtleBot3 model as an environment variable
#         SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),

#         # Launch the TurtleBot3 navigation stack with the provided map
# ExecuteProcess(
#     cmd=[
#         'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
#         f'map:={map_file}',  # Pass the map explicitly
#         'use_sim_time:=True',
#         'use_rviz:=False'  # Disable RViz in the navigation launch file
#     ],
#     output='screen'
# ),

#         Node(
#             package='robot_localization',
#             executable='ekf_node',  # Update to use ekf_node instead
#             name='odom_ekf',
#             output='screen',
#             parameters=[os.path.join(config_dir, 'odom_ekf.yaml')]
#         ),
#         Node(
#             package='robot_localization',
#             executable='ekf_node',  # Update to use ekf_node instead
#             name='map_ekf',
#             output='screen',
#             parameters=[os.path.join(config_dir, 'map_ekf.yaml')]
#         ),

# # ExecuteProcess(
# #     cmd=[
# #         'ros2', 'topic', 'pub', '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
# #         '{header: {frame_id: "map"}, pose: {pose: {position: {x: -2.079555, y: -0.496977, z: 0.0},'
# #         ' orientation: {x: 0.0, y: 0.0, z: 0.026655, w: 0.999645}},'
# #         ' covariance: [0.5, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1]}}'
# #     ],
# #     shell=True,
# #     output='screen'
# # ),


#         # Delay the Gazebo launch to ensure that navigation stack loads first
#         TimerAction(
#             period=5.0,  # Wait 5 seconds before starting Gazebo
#             actions=[
#                 ExecuteProcess(
#                     cmd=[
#                         'ros2', 'launch', 'turtlebot3_gazebo', 'farmWith3CropRows.launch.py'
#                     ],
#                     output='screen'
#                 )
#             ]
#         ),

#         # Delay teleop node to start after Gazebo
#         TimerAction(
#             period=10.0,  # Wait 10 seconds to allow Gazebo to fully load
#             actions=[
#                 ExecuteProcess(
#                     cmd=[
#                         'ros2', 'run', 'turtlebot3_teleop', 'teleop_keyboard'
#                     ],
#                     output='screen'
#                 )
#             ]
#         ),

#         # Delay the custom cylinder node to start last
#         TimerAction(
#             period=15.0,  # Wait 15 seconds to allow everything else to start
#             actions=[
#                 ExecuteProcess(
#                     cmd=[
#                         'ros2', 'run', 'cylindercircling', 'main_node'
#                     ],
#                     output='screen'
#                 )
#             ]
#         ),

#         # Start RViz with the custom configuration file
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             output='screen',
#             arguments=['-d', rviz_config_path]
#         )
#     ])




from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Path to your map
    map_file = os.path.expanduser('/home/andrew/ros2_ws/RTAB_MAPS/farmWith3CropRows/Sprint4Map.yaml')
    # Path to your custom RViz config file
    rviz_config_path = os.path.expanduser('~/ros2_ws/custom_config.rviz')

    config_dir = os.path.join(os.path.expanduser('~/ros2_ws/src/cylindercircling/src/config'))

    return LaunchDescription([

        # Set the TurtleBot3 model as an environment variable
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),

        # Launch the TurtleBot3 navigation stack with the provided map
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
                'map:={}'.format(map_file),  # Fixed format for map argument
                'use_sim_time:=True',
                # 'use_rviz:=False'  # Disable RViz in the navigation launch file
            ],
            output='screen'
        ),

        # # Node for odom EKF
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='odom_ekf',
        #     output='screen',
        #     parameters=[os.path.join(config_dir, 'odom_ekf.yaml')]
        # ),

        # # Node for map EKF
        # Node(
        #     package='robot_localization',
        #     executable='ekf_node',
        #     name='map_ekf',
        #     output='screen',
        #     parameters=[os.path.join(config_dir, 'map_ekf.yaml')]
        # ),

        # # Optional: Initial pose publisher if needed for localization
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'topic', 'pub', '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
        #         '{header: {frame_id: "map"}, pose: {pose: {position: {x: -2.079555, y: -0.496977, z: 0.0},'
        #         ' orientation: {x: 0.0, y: 0.0, z: 0.026655, w: 0.999645}},'
        #         ' covariance: [0.5, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.1]}}'
        #     ],
        #     shell=True,
        #     output='screen'
        # ),

        # Delay the Gazebo launch to ensure that the navigation stack loads first
        TimerAction(
            period=5.0,  # Wait 5 seconds before starting Gazebo
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'launch', 'turtlebot3_gazebo', 'farmWith3CropRows.launch.py'
                    ],
                    output='screen'
                )
            ]
        ),

    

        # Delay the custom cylinder node to start last
        TimerAction(
            period=15.0,  # Wait 15 seconds to allow everything else to start
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'cylindercircling', 'main_node'
                    ],
                    output='screen'
                )
            ]
        ),

        # SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),

        # # Start RViz with the custom configuration file
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', rviz_config_path]
        # )
    ])



# from launch import LaunchDescription
# from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction, IncludeLaunchDescription
# from launch.substitutions import PathJoinSubstitution
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory  # Import this for get_package_share_directory function
# import os

# def generate_launch_description():
#     # Path to the map file
#     map_file = os.path.expanduser('~/ros2_ws/RTAB_MAPS/farmWith3CropRows/Sprint4Map.yaml')
    
#     # AMCL configuration file
#     amcl_params = PathJoinSubstitution([FindPackageShare('nav2_amcl'), 'config', 'amcl_params.yaml'])

#     return LaunchDescription([
#         # Set the TurtleBot3 model as an environment variable
#         SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),

#         # Include the existing robot_localization launch file
#         IncludeLaunchDescription(
#             PythonLaunchDescriptionSource(
#                 os.path.join(get_package_share_directory('robot_localization'), 'launch', 'ekf.launch.py')
#             ),
#             launch_arguments={'use_sim_time': 'true'}.items()
#         ),

#         # Launch AMCL for map-based localization
#         Node(
#             package='nav2_amcl',
#             executable='amcl',
#             name='amcl',
#             output='screen',
#             parameters=[amcl_params, {'use_sim_time': True}]
#         ),

#         # Launch the navigation stack
#         ExecuteProcess(
#             cmd=[
#                 'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
#                 f'map:={map_file}',  # Specify the map file
#                 'use_sim_time:=True'
#             ],
#             output='screen'
#         ),

#         # Delay the Gazebo launch to ensure that navigation stack loads first
#         TimerAction(
#             period=5.0,  # Wait 5 seconds before starting Gazebo
#             actions=[
#                 ExecuteProcess(
#                     cmd=[
#                         'ros2', 'launch', 'turtlebot3_gazebo', 'farmWith3CropRows.launch.py'
#                     ],
#                     output='screen'
#                 )
#             ]
#         ),

#         # Start RViz with a configuration file
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             output='screen',
#             arguments=['-d', PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'rviz', 'nav2_default_view.rviz'])]
#         ),
#     ])
