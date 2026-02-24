.. _doc_tutorials_nav2_goal_pose:

2D Goal Pose Navigation
========================

With the particle filter localizing the car and Nav2 running, you can send the car to any point on the map directly from RViz2.

Prerequisites
-------------

All three of the following must be running:

- **Terminal 1** — ``bringup``
- **Terminal 2** — ``ros2 launch particle_filter localize_launch.py``
- **Terminal 3** — Nav2 (see :ref:`doc_tutorials_nav2_setup`)
- **RViz2** — initial pose already set with **2D Pose Estimate**

Steps
-----

1️⃣ Add Path Visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^

In RViz2, add a **Path** display so you can see the planned route:

- Click **Add** → select **Path**
- Set Topic to ``/plan``

2️⃣ Send a 2D Goal Pose
^^^^^^^^^^^^^^^^^^^^^^^^^

- In the RViz2 toolbar, click **2D Goal Pose**
- Click on the map at the destination
- Drag to set the desired heading, then release

The planned path will appear on the map and the car will begin driving toward the goal.

3️⃣ Watch the Car Navigate
^^^^^^^^^^^^^^^^^^^^^^^^^^^

- The global planner computes a path from the car's current pose to the goal
- The controller follows that path in real time using the localized pose from the particle filter
- The car will stop when it reaches the goal

.. note::

   If the car does not move after setting a goal, confirm that:

   - The particle filter initial pose is set (particles are converged, not spread across the map)
   - Nav2 lifecycle nodes are all active (check ``ros2 node list``)
   - The ``/goal_pose`` topic is being published (check ``ros2 topic echo /goal_pose``)

Topics
------

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - Topic
     - Type
     - Description
   * - ``/goal_pose``
     - ``geometry_msgs/PoseStamped``
     - Published by RViz2 when you click 2D Goal Pose
   * - ``/plan``
     - ``nav_msgs/Path``
     - Global path planned by Nav2
   * - ``/cmd_vel``
     - ``geometry_msgs/Twist``
     - Velocity commands sent to the car by the controller
