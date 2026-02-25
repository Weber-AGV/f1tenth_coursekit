.. _doc_tutorials_nav2_waypoint_nav:

Navigate Through Poses
=======================

Send the car through multiple waypoints in sequence using the Nav2 panel in RViz2.

In the previous tutorial you sent a single 2D Goal Pose. Here you will queue up several waypoints on the map and have Nav2 drive the car through all of them in order.

.. tip::

   If you still have ``bringup`` and Nav2 running from :ref:`doc_tutorials_nav2_goal_pose`, you can skip steps 1️⃣ and 2️⃣ and go straight to 3️⃣.

Steps
-----

1️⃣ Start Bringup (Terminal 1)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Make sure the PlayStation controller is connected to the car, then open a terminal on the robot and launch the car's sensors and drivers:

.. code-block:: bash

   cd ~/f1tenth_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash

.. code-block:: bash

   ros2 launch f1tenth_stack bringup_launch.py

Or, if you have the alias configured: ``bringup``

.. note::

   If Nav2 later reports ``Timed out waiting for transform from base_link to odom``, the PlayStation controller is likely not connected. The VESC driver requires the joystick to fully initialize, and without it the ``odom`` frame is never published.

Leave this terminal running.

2️⃣ Launch Nav2 (Terminal 2)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open a **new** terminal and source the workspace:

.. code-block:: bash

   cd ~/f1tenth_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash

Launch the full Nav2 stack:

.. code-block:: bash

   ros2 launch nav2_bringup bringup_launch.py \
     use_sim_time:=False \
     map:=$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/lab_map.yaml \
     params_file:=$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/nav2_params.yaml

Leave this terminal running.

3️⃣ Open RViz2
^^^^^^^^^^^^^^^^

On your laptop, open RViz2:

.. code-block:: bash

   rviz2

Add a **Map** display (Topic: ``/map``, Durability Policy: ``Transient Local``) to confirm the map is visible.

4️⃣ Add Path Visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^

In RViz2, add the following displays:

- Click **Add** → select **Path** → set Topic to ``/plan``
- Click **Add** → select **Path** → set Topic to ``/local_plan``
- Click **Add** → select **MarkerArray** → set Topic to ``/waypoints``

5️⃣ Add the Nav2 Panel
^^^^^^^^^^^^^^^^^^^^^^^^

In RViz2, open **Panels** → **Add New Panel** → select **Nav2** (from ``nav2_rviz_plugins``).

A new panel will appear at the bottom of RViz2 with navigation controls.

6️⃣ Enable Waypoint Mode
^^^^^^^^^^^^^^^^^^^^^^^^^^

In the Nav2 panel, check the box labeled **Waypoint / Nav Through Poses Mode**.

This changes the behavior of the 2D Goal Pose tool — instead of sending each goal immediately, it will queue waypoints for batch navigation.

7️⃣ Set Waypoints
^^^^^^^^^^^^^^^^^^^

Use the **2D Goal Pose** tool in the RViz2 toolbar to place multiple waypoints on the map:

- Click on the map at the first waypoint location, drag to set heading, release
- Repeat for each additional waypoint
- Each waypoint appears as a numbered marker on the map

Place 3–5 waypoints around the track to start.

8️⃣ Start Navigation
^^^^^^^^^^^^^^^^^^^^^^

In the Nav2 panel, click **Start Nav Through Poses**.

The car will begin driving through each waypoint in the order you placed them. The global planner computes a path to each waypoint in sequence, and the controller follows each path segment.

9️⃣ Watch the Car Navigate
^^^^^^^^^^^^^^^^^^^^^^^^^^^

- The car drives to the first waypoint, then replans to the next
- Watch the ``/plan`` and ``/local_plan`` displays update as the car progresses
- The car stops after reaching the final waypoint

.. note::

   If the car does not move after clicking Start Nav Through Poses, confirm that:

   - Nav2 lifecycle nodes are all active (check ``ros2 node list``)
   - You placed at least one waypoint before clicking start
   - The map is visible in RViz2 (set Durability Policy to ``Transient Local``)

   To clear all waypoints and start over, click **Cancel** in the Nav2 panel, then place new waypoints.

Topics
------

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - Topic
     - Type
     - Description
   * - ``/plan``
     - ``nav_msgs/Path``
     - Global path planned by Nav2 (updates for each waypoint segment)
   * - ``/local_plan``
     - ``nav_msgs/Path``
     - Local path the controller is currently following
   * - ``/waypoints``
     - ``visualization_msgs/MarkerArray``
     - Waypoint markers shown on the map
   * - ``/cmd_vel``
     - ``geometry_msgs/Twist``
     - Velocity commands sent to the car by the controller
