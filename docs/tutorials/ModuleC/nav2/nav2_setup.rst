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

**Command pipeline:** Nav2's controller publishes ``Twist`` on ``/cmd_vel``. The ``cmd_vel_to_ackermann`` node converts this to ``AckermannDriveStamped`` on ``/drive``. The ackermann mux forwards ``/drive`` to the VESC, which drives the wheels.

.. code-block:: text

   controller_server → /cmd_vel (Twist)
     → cmd_vel_to_ackermann → /drive (AckermannDriveStamped)
       → ackermann_mux (priority 10) → VESC

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
- **Nav2 installed** — completed the :ref:`doc_setup_nav2` setup (packages + parameters file)

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

   ros2 launch f1tenth_stack nav2_launch.py

This launches AMCL (localization), the map server, planner, controller, and behavior tree navigator.

.. note::

   **Shutting down Nav2:** The Nav2 component container ignores repeated ``Ctrl+C`` (SIGINT). If Nav2 does not stop after pressing ``Ctrl+C``, use ``Ctrl+\`` (SIGQUIT) instead.

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
