.. _doc_tutorials_nav2_setup:

Nav2 Setup
===========

Once a localization or SLAM node is providing the robot's pose, you can add the **Nav2 navigation stack** to enable point‑and‑click autonomous navigation using the 2D Goal Pose tool in RViz2.

How It Works
------------

The 2D Goal Pose button in RViz2 publishes a ``geometry_msgs/PoseStamped`` message to the ``/goal_pose`` topic. Nav2's **BT Navigator** subscribes to this topic, plans a path using the **Planner Server**, and drives the car to the goal using the **Controller Server**.

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Nav2 Component
     - Role
   * - ``amcl``
     - Localizes the car on the map (Monte Carlo localization)
   * - ``planner_server``
     - Global path planning (A* / NavFn)
   * - ``controller_server``
     - Local path following — publishes ``/cmd_vel`` (Twist)
   * - ``bt_navigator``
     - Subscribes to ``/goal_pose`` and orchestrates planning + control
   * - ``cmd_vel_to_ackermann``
     - Converts ``/cmd_vel`` (Twist) → ``/drive`` (AckermannDriveStamped)
   * - ``lifecycle_manager``
     - Manages the lifecycle of all Nav2 nodes

What is AMCL?
^^^^^^^^^^^^^

**AMCL** (Adaptive Monte Carlo Localization) is a particle filter that answers the question: *"Where am I on this map?"*

It works by maintaining a cloud of **particles** — hundreds of guesses of where the robot might be on the map. Each particle represents a possible pose (x, y, heading). On every laser scan, AMCL compares what the LiDAR actually sees against what it *would* see from each particle's position on the saved map. Particles that match well survive; particles that don't are discarded and replaced. Over time, the cloud converges around the robot's true position.

AMCL publishes the ``map`` → ``odom`` TF transform, which is how the rest of Nav2 knows the robot's position on the map. Without AMCL, the ``map`` frame does not exist and navigation cannot work.

**Why do I need to set a 2D Pose Estimate?** AMCL needs a starting guess. When Nav2 first launches, the particles are not initialized — AMCL doesn't know where to start looking. Clicking **2D Pose Estimate** in RViz2 gives AMCL an approximate starting position, and it spreads the initial particle cloud around that point. As the robot moves, the cloud narrows and localization becomes accurate.

.. note::

   You can visualize the particle cloud in RViz2 by adding a **ParticleCloud** display (under ``nav2_rviz_plugins``) on the ``/particle_cloud`` topic. A spread-out cloud means AMCL is still converging; a tight cluster means it is confident.

   .. image:: img/particle_cloud.png
      :alt: AMCL particle cloud spread out after initial 2D Pose Estimate
      :width: 80%
      :align: center

   .. image:: img/particle_cloud_tighter.png
      :alt: AMCL particle cloud converged after the car has moved
      :width: 80%
      :align: center

.. note::

   **Command pipeline:** Nav2's controller publishes ``Twist`` on ``/cmd_vel``. The ``cmd_vel_to_ackermann`` node converts this to ``AckermannDriveStamped`` on ``/drive``. The ackermann mux forwards ``/drive`` to the VESC, which drives the wheels.

   ``controller_server`` → ``/cmd_vel`` (Twist) → ``cmd_vel_to_ackermann`` → ``/drive`` (AckermannDriveStamped) → ``ackermann_mux`` (priority 10) → VESC

.. note::

   The default global planner plugin used by ``planner_server`` is
   ``nav2_navfn_planner``. It implements the classic **A\*** search algorithm
   (often referred to as *NavFn* in ROS literature) on the occupancy grid
   (a.k.a. costmap) to compute a low‑cost path from the current pose to the
   goal pose. A\* is optimal and complete when the heuristic is admissible, and
   it expands nodes in order of increasing ``f = g + h``. NavFn is simply the
   A\* implementation tuned for navigation maps; you can swap in alternate
   planners (e.g. SMAC) by changing the plugin in your Nav2 parameters.

   For the purposes of the tutorial you can treat the planner as a
   "black box" that returns a sequence of waypoints—the important part is that
   the controller server follows whatever path it provides.

Prerequisites
-------------

Before starting, make sure you have:

- **A saved map** — completed the SLAM tutorial with ``lab_map.pgm`` and ``lab_map.yaml`` in ``~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/``
- **Nav2 installed** — completed the :ref:`doc_setup_nav2` setup
- **Workspace rebuilt after saving the map** — Nav2 reads map files from the install directory, not the source directory. If you haven't rebuilt since saving your map, run:

  .. code-block:: bash

     cd ~/f1tenth_ws
     colcon build --packages-select f1tenth_stack
     source install/setup.bash

Steps
-----

1️⃣ Start Bringup (Terminal 1)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Make sure the PlayStation controller is connected to the car, then open a terminal on the robot and run:

.. code-block:: bash

   bringup

This calls ``ros2 launch f1tenth_stack bringup_launch.py``, which starts the car's sensors and drivers.

.. note::

   If Nav2 later reports ``Timed out waiting for transform from base_link to odom``, the PlayStation controller is likely not connected. The VESC driver requires the joystick to fully initialize, and without it the ``odom`` frame is never published.

.. warning::

   **Hold R1 for autonomous mode.** By default the joystick continuously publishes zero-speed commands at high priority, blocking Nav2. Hold **R1** (button 5) on the PlayStation controller to enable autonomous mode — this lets Nav2's drive commands through. Releasing R1 returns to manual joystick control.

Leave this terminal running.

2️⃣ Launch Nav2 (Terminal 2)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open a **new** terminal and source the workspace:

.. code-block:: bash

   cd ~/f1tenth_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash

Launch the full Nav2 stack (AMCL + map server + navigation):

.. code-block:: bash

   ros2 launch f1tenth_stack nav2_launch.py map_name:=hallway_map

The ``map_name`` argument tells Nav2 which map file to load from ``f1tenth_stack/maps/``. Change ``hallway_map`` to match the name you used when saving your map (without the file extension).

This launches the full Nav2 stack:

- **map_server** — loads your saved map and publishes the occupancy grid
- **AMCL** — localizes the car on the map using a particle filter (publishes the ``map`` → ``odom`` TF)
- **planner_server** — computes a global path from the car to the goal (A*)
- **controller_server** — follows the planned path by publishing velocity commands
- **behavior_server** — handles recovery behaviors (spin, backup, wait) when the car gets stuck
- **bt_navigator** — the behavior tree that orchestrates planning, control, and recovery into a complete navigation loop
- **cmd_vel_to_ackermann** — converts Nav2's ``/cmd_vel`` (Twist) to ``/drive`` (AckermannDriveStamped) for the F1TENTH

.. note::

   **Shutting down Nav2:** The Nav2 component container ignores repeated ``Ctrl+C`` (SIGINT). If Nav2 does not stop after pressing ``Ctrl+C``, use ``Ctrl+\`` (SIGQUIT) instead.

.. note::

   If map_server fails with ``Failed processing YAML file ... for reason: bad file``, the map file in the install directory is empty or corrupted. Check the source file:

   .. code-block:: bash

      cat ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/lab_map.yaml

   If the file has valid content, rebuild with ``colcon build --packages-select f1tenth_stack`` and relaunch. If the file is empty or missing, go back to the SLAM tutorial and save the map again, then rebuild.

Leave this terminal running.

3️⃣ Verify Nav2 is Running (Terminal 3)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open a **third** terminal and check that the Nav2 lifecycle nodes are active:

.. code-block:: bash

   ros2 node list

You should see these Nav2 nodes:

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Node
     - Role
   * - ``/map_server``
     - Loads and publishes the occupancy grid map
   * - ``/amcl``
     - Localizes the car on the map (Monte Carlo localization)
   * - ``/planner_server``
     - Computes a global path from the car's pose to the goal (A*)
   * - ``/controller_server``
     - Follows the planned path by sending velocity commands to the car
   * - ``/bt_navigator``
     - Orchestrates planning and control via a behavior tree

Confirm the planner is ready by checking for the ``/plan`` topic:

.. code-block:: bash

   ros2 topic list | grep plan

Applying These Changes to Other Robots
---------------------------------------

The Nav2 setup lives in two files within the git-tracked workspace. The fastest way to set up additional robots is to copy and paste the file contents directly.

**nav2_launch.py**

On the other robot, open the launch file:

.. code-block:: bash

   code ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/launch/nav2_launch.py

Replace the entire contents with:

.. code-block:: python

   # MIT License

   # Copyright (c) 2025 WSU F1TENTH

   # Permission is hereby granted, free of charge, to any person obtaining a copy
   # of this software and associated documentation files (the "Software"), to deal
   # in the Software without restriction, including without limitation the rights
   # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   # copies of the Software, and to permit persons to whom the Software is
   # furnished to do so, subject to the following conditions:

   # The above copyright notice and this permission notice shall be included in all
   # copies or substantial portions of the Software.

   # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   # SOFTWARE.

   from launch import LaunchDescription
   from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
   from launch.launch_description_sources import PythonLaunchDescriptionSource
   from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
   from launch_ros.actions import Node
   from launch_ros.substitutions import FindPackageShare
   from ament_index_python.packages import get_package_share_directory
   import os


   def generate_launch_description():
       f1tenth_stack_dir = get_package_share_directory('f1tenth_stack')
       nav2_bringup_dir = get_package_share_directory('nav2_bringup')

       map_name_arg = DeclareLaunchArgument(
           'map_name',
           default_value='lab_map',
           description='Map name (without extension) in f1tenth_stack/maps/'
       )

       nav2_params = os.path.join(f1tenth_stack_dir, 'config', 'nav2_params.yaml')
       map_file = PathJoinSubstitution(
           [FindPackageShare('f1tenth_stack'), 'maps', [LaunchConfiguration('map_name'), '.yaml']]
       )

       nav2_bringup = IncludeLaunchDescription(
           PythonLaunchDescriptionSource(
               os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
           ),
           launch_arguments={
               'map': map_file,
               'params_file': nav2_params,
               'use_sim_time': 'False',
               'slam': 'False',     # use AMCL for localization, not slam_toolbox
               'autostart': 'True',
           }.items()
       )

       # Converts nav2 cmd_vel (Twist) → drive (AckermannDriveStamped) for the mux
       cmd_vel_to_ackermann_node = Node(
           package='f1tenth_stack',
           executable='cmd_vel_to_ackermann',
           name='cmd_vel_to_ackermann',
           parameters=[{'wheelbase': 0.25}]
       )

       return LaunchDescription([map_name_arg, nav2_bringup, cmd_vel_to_ackermann_node])

**localize_launch.py** (particle filter fix)

On the other robot, open the particle filter launch file:

.. code-block:: bash

   code ~/f1tenth_ws/src/particle_filter/launch/localize_launch.py

Replace the entire contents with:

.. code-block:: python

   # MIT License

   # Copyright (c) 2020 Hongrui Zheng

   # Permission is hereby granted, free of charge, to any person obtaining a copy
   # of this software and associated documentation files (the "Software"), to deal
   # in the Software without restriction, including without limitation the rights
   # to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
   # copies of the Software, and to permit persons to whom the Software is
   # furnished to do so, subject to the following conditions:

   # The above copyright notice and this permission notice shall be included in all
   # copies or substantial portions of the Software.

   # THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   # IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   # FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   # AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   # LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
   # OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   # SOFTWARE.

   from launch import LaunchDescription
   from launch_ros.actions import Node
   from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
   from launch.actions import DeclareLaunchArgument
   from launch_ros.substitutions import FindPackageShare
   from ament_index_python.packages import get_package_share_directory
   import os
   import yaml

   def generate_launch_description():
       # config and args
       localize_config = os.path.join(
           get_package_share_directory('particle_filter'),
           'config',
           'localize.yaml'
       )
       localize_config_dict = yaml.safe_load(open(localize_config, 'r'))
       map_name = localize_config_dict['map_server']['ros__parameters']['map']
       localize_la = DeclareLaunchArgument(
           'localize_config',
           default_value=localize_config,
           description='Localization configs')
       map_name_la = DeclareLaunchArgument(
           'map_name',
           default_value=map_name,
           description='Map name (without extension) in particle_filter/maps/')
       ld = LaunchDescription([localize_la, map_name_la])

       # nodes
       pf_node = Node(
           package='particle_filter',
           executable='particle_filter',
           name='particle_filter',
           parameters=[LaunchConfiguration('localize_config')]
       )
       map_server_node = Node(
           package='nav2_map_server',
           executable='map_server',
           name='map_server',
           parameters=[{'yaml_filename': PathJoinSubstitution(
                            [FindPackageShare('particle_filter'), 'maps',
                             [LaunchConfiguration('map_name'), '.yaml']])},
                       {'topic': 'map'},
                       {'frame_id': 'map'},
                       {'output': 'screen'},
                       {'use_sim_time': True}]
       )
       nav_lifecycle_node = Node(
           package='nav2_lifecycle_manager',
           executable='lifecycle_manager',
           name='lifecycle_manager_localization',
           output='screen',
           parameters=[{'use_sim_time': True},
                       {'autostart': True},
                       {'node_names': ['map_server']}]
       )

       # finalize
       ld.add_action(nav_lifecycle_node)
       ld.add_action(map_server_node)
       ld.add_action(pf_node)

       return ld

**Rebuild and source**

After pasting both files, rebuild and source the workspace:

.. code-block:: bash

   cd ~/f1tenth_ws
   colcon build --packages-select f1tenth_stack particle_filter
   source install/setup.bash
