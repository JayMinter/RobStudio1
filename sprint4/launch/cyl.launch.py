from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess, TimerAction
import os

def generate_launch_description():
    # Path to your map
    map_file = os.path.expanduser('/home/andrew/ros2_ws/RTAB_MAPS/farmWith3CropRows/Sprint4Map.yaml')

    return LaunchDescription([

        # Set the TurtleBot3 model as an environment variable
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi'),

        # Launch the TurtleBot3 navigation stack with the provided map
        ExecuteProcess(
            cmd=[
                'ros2', 'launch', 'turtlebot3_navigation2', 'navigation2.launch.py',
                f'map:={map_file}',  # Pass the map explicitly
                'use_sim_time:=True'
            ],
            output='screen'
        ),

        
        # Delay the Gazebo launch to ensure that navigation stack loads first
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

    

        # Delay the main node to start after everything else
        TimerAction(
            period=10.0,  # Wait 15 seconds to allow everything else to start
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'sprint4', 'mainrun'
                    ],
                    output='screen'
                )
            ]
        ),
    ])
