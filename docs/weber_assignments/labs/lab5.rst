.. _doc_weber_lab5:


Lab 5 - SLAM and Nav2 Navigation
==================================

I. Learning Goals
-----------------

- Map an environment using SLAM Toolbox
- Understand occupancy grid maps (``.pgm`` / ``.yaml``)
- Localize the car on a saved map using AMCL
- Navigate autonomously using the Nav2 stack (2D Goal Pose and Waypoint Navigation)
- Understand the Nav2 command pipeline: planner → controller → cmd_vel → ackermann → drive

II. Overview
------------

In this lab you will use the RoboRacer to map the CAE hallway loop, save the map, and then use the Nav2 navigation stack to autonomously drive the car to goal poses on that map. This lab ties together SLAM (mapping), AMCL (localization), and Nav2 (path planning + control).

.. list-table::
   :header-rows: 1
   :widths: 15 40 45

   * - Part
     - Task
     - What You Learn
   * - II
     - Map the hallway with SLAM Toolbox
     - Occupancy grids, SLAM, map saving
   * - III
     - Send a 2D Goal Pose
     - AMCL localization, Nav2 path planning, autonomous driving
   * - IV
     - Navigate through multiple waypoints
     - Waypoint sequencing, behavior trees, Nav2 panel

III. Prerequisites
------------------

Before starting this lab, make sure you have:

- Completed the :ref:`SLAM tutorial <doc_tutorials_slam_map>` (you know how to drive, save, and verify a map)
- Completed the :ref:`Nav2 setup <doc_setup_nav2>` (Nav2 packages installed, parameters file, launch file, and converter node created)

IV. Part 1 — Map the Environment
----------------------------------

Use SLAM Toolbox to create a map of the CAE hallway loop.

1. Connect the PlayStation controller and start **bringup** on the car.
2. Launch **SLAM Toolbox** in a second terminal.
3. Open **RViz2** and drive the car through the entire hallway loop using the joystick. Make sure the loop is closed.
4. Save the map:

   .. code-block:: bash

      mkdir -p ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps
      ros2 run nav2_map_server map_saver_cli -f ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/lab_map

5. Verify the files exist (``lab_map.pgm`` and ``lab_map.yaml``).
6. Rebuild the workspace so Nav2 can find the map:

   .. code-block:: bash

      cd ~/f1tenth_ws
      colcon build --packages-select f1tenth_stack
      source install/setup.bash

7. **Verify** the map loads correctly by running the standalone map server and checking it in RViz2 (see the :ref:`SLAM tutorial <doc_tutorials_slam_map>` for detailed steps).

.. tip::

   Refer to the :ref:`Saving the Map <doc_tutorials_slam_map>` tutorial for the full step-by-step walkthrough if needed.

V. Part 2 — 2D Goal Pose Navigation
-------------------------------------

Use Nav2 to send the car to a single goal on the map.

1. Close any running SLAM, map server, or RViz2 processes from Part 1. Start fresh.
2. Start **bringup** (Terminal 1):

   .. code-block:: bash

      bringup

3. Launch **Nav2** (Terminal 2):

   .. code-block:: bash

      cd ~/f1tenth_ws
      source /opt/ros/humble/setup.bash
      source install/setup.bash
      ros2 launch f1tenth_stack nav2_launch.py

4. Open **RViz2** (Terminal 3) and add the following displays:

   - **Map** — Topic: ``/map``, Durability Policy: ``Transient Local``
   - **LaserScan** — Topic: ``/scan``
   - **PoseArray** — Topic: ``/particle_cloud`` (shows AMCL particle spread)

5. Set the **2D Pose Estimate** — click the button in RViz2 and place the arrow at the car's actual position and orientation on the map. Watch the AMCL particle cloud converge around the car.
6. Set a **2D Goal Pose** — click the button and place a goal on the map.

.. warning::

   **Hold R1 on the PlayStation controller** while the car is driving autonomously. Releasing R1 returns to manual joystick control and stops Nav2's commands from reaching the wheels.

7. Watch the car plan a path (green line in RViz2) and drive to the goal.
8. Send at least **3 different goals** to different locations on the map.

.. tip::

   Refer to the :ref:`2D Goal Pose <doc_tutorials_nav2_goal_pose>` tutorial for the full walkthrough.

VI. Part 3 — Waypoint Navigation
----------------------------------

Use Nav2's Navigate Through Poses to send the car through multiple waypoints in sequence.

1. With bringup and Nav2 still running, open the **Nav2 panel** in RViz2 (Panels → Add New Panel → ``nav2_rviz_plugins/Nav2 Panel``).
2. Set the **2D Pose Estimate** if not already set.
3. Switch to **Waypoint / Nav Through Poses Mode** in the Nav2 panel.
4. Click multiple points on the map to queue up waypoints.
5. Click **Start Nav Through Poses** in the Nav2 panel.

.. warning::

   **Hold R1** throughout the entire waypoint sequence. If you release R1 mid-run, the car stops and Nav2 may report a controller failure.

6. Create a route of at least **5 waypoints** that makes the car drive a loop through the hallway.

.. tip::

   Refer to the :ref:`Navigate Through Poses <doc_tutorials_nav2_waypoint_nav>` tutorial for the full walkthrough.

VII. Deliverables
------------------

.. list-table::
   :header-rows: 1
   :widths: 10 60 30

   * - #
     - Deliverable
     - Points
   * - 1
     - Map files (``lab_map.pgm`` and ``lab_map.yaml``) submitted to Canvas
     - 20
   * - 2
     - Screenshot of RViz2 showing the loaded map with AMCL particle cloud after setting the initial pose
     - 20
   * - 3
     - Video: 2D Goal Pose — car navigating to a goal on the map (screen recording of RViz2 showing path + car movement)
     - 20
   * - 4
     - Video: Waypoint Navigation — car following a 5+ waypoint route through the hallway (screen recording of RViz2)
     - 20
   * - 5
     - In-class demonstration to the instructor
     - 20

**Total: 100 Points**
