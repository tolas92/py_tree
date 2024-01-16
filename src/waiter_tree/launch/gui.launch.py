import os
import time
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Create the LaunchDescription object
    ld = LaunchDescription()
    
    customer_menu_node = Node(
        package='waiter_tree',
        executable='customer_menu.py',
        output='screen'
    )
    kitchen_table_node=Node(
        package='waiter_tree',
        executable='kitchen_table.py',
        output='screen'
    )

    table_nav_node=Node(
        package='navigator_planner',
        executable='table_navigation.py',
        output='screen'
    )

    waiter_node=Node(
        package='waiter_tree',
        executable='waiter.py',
        output='screen'
    )




    ld.add_action(customer_menu_node)
   # ld.add_action(TimerAction(period=2.0,actions=[kitchen_table_node]))
    ld.add_action(TimerAction(period=4.0,actions=[table_nav_node]))
    ld.add_action(TimerAction(period=7.0,actions=[waiter_node]))

    return ld
 