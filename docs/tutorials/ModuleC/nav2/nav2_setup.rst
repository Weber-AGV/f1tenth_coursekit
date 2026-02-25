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
   * - ``planner_server``
     - Global path planning (A* / NavFn)
   * - ``controller_server``
     - Local path following (drives the car along the planned path)
   * - ``bt_navigator``
     - Subscribes to ``/goal_pose`` and orchestrates planning + control
   * - ``lifecycle_manager``
     - Manages the lifecycle of all Nav2 nodes


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

Before launching Nav2, the following must already be running:

- **Terminal 1** — ``bringup`` (sensors + drivers)
- **Terminal 2** — a localization or SLAM node providing a map and pose.
  For example, you might run the coursekit particle filter launch or a SLAM
  package such as ``slam_toolbox``
  (``ros2 launch slam_toolbox online_async_launch.py``) to build the map on the
  fly.  As long as the node publishes a ``map`` on ``/map`` and broadcasts
  the robot's pose in the map frame (via TF), Nav2 will work.
- **RViz2** — initial pose set with **2D Pose Estimate**

.. note::

   Some localization launches (such as the coursekit particle filter demo)
   already handle the map server and lifecycle transitions, in which case you
   can safely set ``map_server:=False``.  When using a separate SLAM node you
   can either let that node provide the map (``map_server:=False``) or run
   Nav2's map server with a previously saved map by setting
   ``map_server:=True`` and pointing to the map file.


Create the Parameters File
---------------------------

Nav2's built-in defaults are tuned for a Turtlebot3 (differential drive, 0.26 m/s max speed, wrong robot radius). You need a parameters file tuned for the F1TENTH Ackermann platform.

Create the file on the robot:

.. code-block:: bash

   nano ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/nav2_params.yaml

Paste the contents of the F1TENTH Nav2 parameters file: :download:`nav2_params.yaml`

Key differences from the Turtlebot3 defaults:

.. list-table::
   :header-rows: 1
   :widths: 30 25 25 20

   * - Parameter
     - Turtlebot3 Default
     - F1TENTH Value
     - Why
   * - Controller plugin
     - DWB (diff-drive)
     - RegulatedPurePursuit
     - Ackermann steering
   * - ``desired_linear_vel``
     - 0.26 m/s
     - 1.0 m/s
     - F1TENTH is faster
   * - ``robot_radius``
     - 0.22 m
     - 0.18 m
     - F1TENTH footprint
   * - ``inflation_radius``
     - 0.55 m
     - 0.30 m
     - Narrower car, tighter paths
   * - ``use_sim_time``
     - True
     - False
     - Running on real hardware


Launch Nav2
-----------

Open a new terminal (Terminal 3):

.. code-block:: bash

   cd ~/f1tenth_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash

.. code-block:: bash

   ros2 launch nav2_bringup navigation_launch.py \
     use_sim_time:=False \
     params_file:=$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/nav2_params.yaml

Verify Nav2 is Running
-----------------------

Check that the Nav2 lifecycle nodes are active:

.. code-block:: bash

   ros2 node list

You should see nodes including ``/planner_server``, ``/controller_server``, and ``/bt_navigator``.

Confirm the planner is ready by checking for the ``/plan`` topic:

.. code-block:: bash

   ros2 topic list | grep plan
