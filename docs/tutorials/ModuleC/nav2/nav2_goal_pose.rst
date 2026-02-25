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

Open a terminal on the robot and launch the car's sensors and drivers:

.. code-block:: bash

   cd ~/f1tenth_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash

.. code-block:: bash

   ros2 launch f1tenth_stack bringup_launch.py

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

5️⃣ Send a 2D Goal Pose
^^^^^^^^^^^^^^^^^^^^^^^^^

- In the RViz2 toolbar, click **2D Goal Pose**
- Click on the map at the destination
- Drag to set the desired heading, then release

The planned path will appear on the map and the car will begin driving toward the goal.

6️⃣ Watch the Car Navigate
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
