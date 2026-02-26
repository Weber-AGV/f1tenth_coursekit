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

Create the Velocity Converter
------------------------------

Nav2's controller publishes ``geometry_msgs/Twist`` on ``/cmd_vel``, but the F1TENTH's ackermann mux expects ``AckermannDriveStamped`` on the ``/drive`` topic. A converter node bridges this gap.

3️⃣ **Create the converter node**

.. code-block:: bash

   nano ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/f1tenth_stack/cmd_vel_to_ackermann.py

Paste the following contents:

.. code-block:: python

   import rclpy
   from rclpy.node import Node
   from geometry_msgs.msg import Twist
   from ackermann_msgs.msg import AckermannDriveStamped
   import math

   class CmdVelToAckermann(Node):
       def __init__(self):
           super().__init__('cmd_vel_to_ackermann')
           self.declare_parameter('wheelbase', 0.25)
           self.wheelbase = self.get_parameter('wheelbase').value

           self.sub = self.create_subscription(Twist, '/cmd_vel', self.callback, 10)
           self.pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)

       def callback(self, msg):
           ack = AckermannDriveStamped()
           ack.header.stamp = self.get_clock().now().to_msg()
           ack.drive.speed = msg.linear.x

           if abs(msg.linear.x) > 0.01:
               ack.drive.steering_angle = math.atan(
                   self.wheelbase * msg.angular.z / msg.linear.x
               )
           else:
               ack.drive.steering_angle = 0.0

           self.pub.publish(ack)

   def main():
       rclpy.init()
       node = CmdVelToAckermann()
       rclpy.spin(node)
       node.destroy_node()
       rclpy.shutdown()

   if __name__ == '__main__':
       main()

Save the file (``Ctrl+O``, Enter, ``Ctrl+X``).

4️⃣ **Add the entry point to setup.py**

Open ``setup.py``:

.. code-block:: bash

   nano ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/setup.py

Find the ``console_scripts`` list inside ``entry_points`` and add a new line:

.. code-block:: python

   'cmd_vel_to_ackermann = f1tenth_stack.cmd_vel_to_ackermann:main',

Save the file.

Create the Launch File
-----------------------

The Nav2 launch file wraps ``nav2_bringup`` with the correct map, parameters, and the velocity converter for the F1TENTH.

5️⃣ **Create the launch file**

.. code-block:: bash

   nano ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/launch/nav2_launch.py

Paste the following contents:

.. code-block:: python

   from launch import LaunchDescription
   from launch.actions import IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch_ros.actions import Node
   from ament_index_python.packages import get_package_share_directory
   import os

   def generate_launch_description():
       f1tenth_dir = get_package_share_directory('f1tenth_stack')
       nav2_bringup_dir = get_package_share_directory('nav2_bringup')

       nav2_bringup = IncludeLaunchDescription(
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

       cmd_vel_converter = Node(
           package='f1tenth_stack',
           executable='cmd_vel_to_ackermann',
           name='cmd_vel_to_ackermann',
           parameters=[{'wheelbase': 0.25}]
       )

       return LaunchDescription([nav2_bringup, cmd_vel_converter])

Save the file (``Ctrl+O``, Enter, ``Ctrl+X``).

6️⃣ **Rebuild the workspace**

.. code-block:: bash

   cd ~/f1tenth_ws
   colcon build --packages-select f1tenth_stack
   source install/setup.bash

7️⃣ **Verify the files exist**

.. code-block:: bash

   ls ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/nav2_params.yaml
   ls ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/f1tenth_stack/cmd_vel_to_ackermann.py
   ls ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/launch/nav2_launch.py
