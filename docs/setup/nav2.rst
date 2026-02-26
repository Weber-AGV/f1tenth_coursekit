.. _doc_setup_nav2:

SETUP - Nav2
==================================

Install Nav2 (Navigation Stack)
---------------------------------

1️⃣ **Install the Nav2 packages**

.. code-block:: console

   sudo apt update
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

``ros-humble-navigation2`` provides the core Nav2 nodes (planner, controller, bt_navigator) and ``ros-humble-nav2-bringup`` provides the launch files.


Create the Parameters File
---------------------------

Nav2 requires a parameters file that configures the planner, controller, and costmaps for the F1TENTH Ackermann platform.

2️⃣ **Create the file on the robot**

.. code-block:: bash

   nano ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/nav2_params.yaml

Paste the following contents:

.. code-block:: yaml

   # =============================================================================
   # Nav2 Parameters — F1TENTH RoboRacer
   # =============================================================================
   # Minimal configuration tuned for the F1TENTH Ackermann platform.
   # Place this file at:
   #   ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/nav2_params.yaml
   # =============================================================================

   map_server:
     ros__parameters:
       use_sim_time: False
       yaml_filename: ""

   bt_navigator:
     ros__parameters:
       use_sim_time: False
       global_frame: map
       robot_base_frame: base_link
       odom_topic: /odom
       bt_loop_duration: 10
       default_server_timeout: 20
       navigators: ["navigate_to_pose", "navigate_through_poses"]
       navigate_to_pose:
         plugin: "nav2_bt_navigator/NavigateToPoseNavigator"
       navigate_through_poses:
         plugin: "nav2_bt_navigator/NavigateThroughPosesNavigator"

   controller_server:
     ros__parameters:
       use_sim_time: False
       controller_frequency: 20.0
       min_x_velocity_threshold: 0.001
       min_y_velocity_threshold: 0.5
       min_theta_velocity_threshold: 0.001
       progress_checker_plugins: ["progress_checker"]
       goal_checker_plugins: ["general_goal_checker"]
       controller_plugins: ["FollowPath"]

       progress_checker:
         plugin: "nav2_controller::SimpleProgressChecker"
         required_movement_radius: 0.5
         movement_time_allowance: 10.0

       general_goal_checker:
         stateful: True
         plugin: "nav2_controller::SimpleGoalChecker"
         xy_goal_tolerance: 0.25
         yaw_goal_tolerance: 0.25

       FollowPath:
         plugin: "nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController"
         desired_linear_vel: 1.0
         lookahead_dist: 1.0
         min_lookahead_dist: 0.5
         max_lookahead_dist: 2.0
         lookahead_time: 1.5
         rotate_to_heading_angular_vel: 1.0
         transform_tolerance: 0.1
         use_velocity_scaled_lookahead_dist: True
         min_approach_linear_velocity: 0.3
         approach_velocity_scaling_dist: 1.0
         use_collision_detection: True
         max_allowed_time_to_collision_up_to_carrot: 1.0
         use_regulated_linear_velocity_scaling: True
         use_cost_regulated_linear_velocity_scaling: False
         regulated_linear_scaling_min_radius: 0.9
         regulated_linear_scaling_min_speed: 0.3
         max_angular_accel: 3.2
         allow_reversing: False

   planner_server:
     ros__parameters:
       use_sim_time: False
       expected_planner_frequency: 20.0
       planner_plugins: ["GridBased"]
       GridBased:
         plugin: "nav2_navfn_planner/NavfnPlanner"
         tolerance: 0.5
         use_astar: True
         allow_unknown: True

   local_costmap:
     local_costmap:
       ros__parameters:
         use_sim_time: False
         update_frequency: 5.0
         publish_frequency: 2.0
         global_frame: odom
         robot_base_frame: base_link
         rolling_window: True
         width: 3
         height: 3
         resolution: 0.05
         robot_radius: 0.18
         plugins: ["obstacle_layer", "inflation_layer"]
         obstacle_layer:
           plugin: "nav2_costmap_2d::ObstacleLayer"
           enabled: True
           observation_sources: scan
           scan:
             topic: /scan
             max_obstacle_height: 2.0
             clearing: True
             marking: True
             data_type: "LaserScan"
             raytrace_max_range: 10.0
             raytrace_min_range: 0.0
             obstacle_max_range: 8.0
             obstacle_min_range: 0.0
         inflation_layer:
           plugin: "nav2_costmap_2d::InflationLayer"
           cost_scaling_factor: 3.0
           inflation_radius: 0.30

   global_costmap:
     global_costmap:
       ros__parameters:
         use_sim_time: False
         update_frequency: 1.0
         publish_frequency: 1.0
         global_frame: map
         robot_base_frame: base_link
         robot_radius: 0.18
         resolution: 0.05
         track_unknown_space: True
         plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
         static_layer:
           plugin: "nav2_costmap_2d::StaticLayer"
           map_subscribe_transient_local: True
         obstacle_layer:
           plugin: "nav2_costmap_2d::ObstacleLayer"
           enabled: True
           observation_sources: scan
           scan:
             topic: /scan
             max_obstacle_height: 2.0
             clearing: True
             marking: True
             data_type: "LaserScan"
             raytrace_max_range: 10.0
             raytrace_min_range: 0.0
             obstacle_max_range: 8.0
             obstacle_min_range: 0.0
         inflation_layer:
           plugin: "nav2_costmap_2d::InflationLayer"
           cost_scaling_factor: 3.0
           inflation_radius: 0.30

   behavior_server:
     ros__parameters:
       use_sim_time: False
       costmap_topic: local_costmap/costmap_raw
       footprint_topic: local_costmap/published_footprint
       cycle_frequency: 10.0
       behavior_plugins: ["wait", "backup"]
       wait:
         plugin: "nav2_behaviors/Wait"
       backup:
         plugin: "nav2_behaviors/BackUp"

   lifecycle_manager:
     ros__parameters:
       use_sim_time: False
       autostart: True
       node_names:
         - map_server
         - controller_server
         - planner_server
         - behavior_server
         - bt_navigator

Save the file (``Ctrl+O``, Enter, ``Ctrl+X``).

Create the Launch File
-----------------------

The Nav2 launch file wraps ``nav2_bringup`` with the correct map and parameters paths for the F1TENTH.

3️⃣ **Create the file on the robot**

.. code-block:: bash

   nano ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/launch/nav2_launch.py

Paste the following contents:

.. code-block:: python

   from launch import LaunchDescription
   from launch.actions import IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from ament_index_python.packages import get_package_share_directory
   import os

   def generate_launch_description():
       f1tenth_dir = get_package_share_directory('f1tenth_stack')
       nav2_bringup_dir = get_package_share_directory('nav2_bringup')

       return LaunchDescription([
           IncludeLaunchDescription(
               PythonLaunchDescriptionSource(
                   os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
               ),
               launch_arguments={
                   'map': os.path.join(f1tenth_dir, 'maps', 'lab_map.yaml'),
                   'params_file': os.path.join(f1tenth_dir, 'config', 'nav2_params.yaml'),
                   'use_sim_time': 'False',
                   'autostart': 'True',
               }.items()
           )
       ])

Save the file (``Ctrl+O``, Enter, ``Ctrl+X``).

4️⃣ **Rebuild the workspace**

.. code-block:: bash

   cd ~/f1tenth_ws
   colcon build --packages-select f1tenth_stack
   source install/setup.bash

5️⃣ **Verify both files exist**

.. code-block:: bash

   ls ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/nav2_params.yaml
   ls ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/launch/nav2_launch.py
