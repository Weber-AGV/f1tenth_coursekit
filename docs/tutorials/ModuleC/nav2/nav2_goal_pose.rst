.. _doc_tutorials_nav2_goal_pose:

2D Goal Pose Navigation
========================

With Nav2 running and the map loaded, you can send the car to any point on the map directly from RViz2.

Prerequisites
-------------

The following must be running:

- **Terminal 1** — ``bringup`` (sensors + drivers)
- **Terminal 2** — Nav2 (see :ref:`doc_tutorials_nav2_setup`)
- **RViz2** — open with the map visible

Steps
-----

1️⃣ Add Path Visualization
^^^^^^^^^^^^^^^^^^^^^^^^^^^

In RViz2, add the following displays to visualize Nav2's planning and control:

**Global Plan** (the planned route from start to goal):

- Click **Add** → select **Path**
- Set Topic to ``/plan``

**Local Plan** (the path the controller is currently following):

- Click **Add** → select **Path**
- Set Topic to ``/local_plan``

**Waypoint Markers** (Nav2 internal markers):

- Click **Add** → select **MarkerArray**
- Set Topic to ``/waypoints``

2️⃣ Send a 2D Goal Pose
^^^^^^^^^^^^^^^^^^^^^^^^^

- In the RViz2 toolbar, click **2D Goal Pose**
- Click on the map at the destination
- Drag to set the desired heading, then release

The planned path will appear on the map and the car will begin driving toward the goal.

3️⃣ Watch the Car Navigate
^^^^^^^^^^^^^^^^^^^^^^^^^^^

- The global planner computes a path from the car's current pose to the goal
- The controller follows that path in real time — watch the ``/local_plan`` display to see the controller's tracking path
- The car will stop when it reaches the goal

.. note::

   If the car does not move after setting a goal, confirm that:

   - Nav2 lifecycle nodes are all active (check ``ros2 node list``)
   - The ``/goal_pose`` topic is being published (check ``ros2 topic echo /goal_pose``)
   - The map is visible in RViz2 (set Durability Policy to ``Transient Local``)

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
   * - ``/local_plan``
     - ``nav_msgs/Path``
     - Local path the controller is currently following
   * - ``/waypoints``
     - ``visualization_msgs/MarkerArray``
     - Nav2 internal waypoint markers
   * - ``/cmd_vel``
     - ``geometry_msgs/Twist``
     - Velocity commands sent to the car by the controller
