.. _doc_tutorials_nav2_goal_pose:

2D Goal Pose Navigation
========================

Send the car to any point on the map using the 2D Goal Pose tool in RViz2.

.. tip::

   If you still have ``bringup`` and Nav2 running from :ref:`doc_tutorials_nav2_setup`, you can skip steps 1️⃣ and 2️⃣ and go straight to 3️⃣.

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

.. tip::

   **No deadman button needed.** Unlike teleop, you do not need to hold the deadman button for Nav2 autonomous navigation — the car will drive itself. If you need to take manual control at any time, hold the deadman button (L1) on the PlayStation controller to override Nav2.

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

   ros2 launch f1tenth_stack nav2_launch.py

Leave this terminal running.

3️⃣ Open RViz2
^^^^^^^^^^^^^^^^

On your laptop, open RViz2:

.. code-block:: bash

   rviz2

Add a **Map** display (Topic: ``/map``, Durability Policy: ``Transient Local``) to confirm the map is visible.

4️⃣ Set Initial Pose
^^^^^^^^^^^^^^^^^^^^^^

Before Nav2 can navigate, AMCL needs to know where the car is on the map.

- In the RViz2 toolbar, click **2D Pose Estimate**
- Click on the map at the car's approximate location
- Drag to set the car's heading, then release

You should see the robot's position update on the map. AMCL will refine the estimate as the car moves.

5️⃣ Add Path Visualization
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

6️⃣ Send a 2D Goal Pose
^^^^^^^^^^^^^^^^^^^^^^^^^

- In the RViz2 toolbar, click **2D Goal Pose**
- Click on the map at the destination
- Drag to set the desired heading, then release

The planned path will appear on the map and the car will begin driving toward the goal.

7️⃣ Watch the Car Navigate
^^^^^^^^^^^^^^^^^^^^^^^^^^^

- The global planner computes a path from the car's current pose to the goal
- The controller follows that path in real time — watch the ``/local_plan`` display to see the controller's tracking path
- The car will stop when it reaches the goal

.. note::

   If the car does not move after setting a goal, confirm that:

   - You set the initial pose with **2D Pose Estimate** (step 4️⃣)
   - The deadman button (L1) is **not held** — holding it overrides Nav2 with manual control
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
