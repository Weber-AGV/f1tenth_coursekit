.. _doc_tutorials_nav2_setup:

Nav2 Setup
===========

With the particle filter localizing the car, you can add the **Nav2 navigation stack** to enable point-and-click autonomous navigation using the 2D Goal Pose tool in RViz2.

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

Prerequisites
-------------

Before launching Nav2, the following must already be running:

- **Terminal 1** — ``bringup`` (sensors + drivers)
- **Terminal 2** — ``ros2 launch particle_filter localize_launch.py`` (map + localization)
- **RViz2** — initial pose set with **2D Pose Estimate**

.. note::

   Do not run ``nav2_map_server`` separately — ``localize_launch.py`` already handles the map server and lifecycle transitions. Nav2 will use the map that is already published on ``/map``.


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
     map_server:=False \
     params_file:=$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/nav2_params.yaml

.. note::

   ``map_server:=False`` tells Nav2 not to launch its own map server since the particle filter launch already provides the map on ``/map``.

Verify Nav2 is Running
-----------------------

Check that the Nav2 lifecycle nodes are active:

.. code-block:: bash

   ros2 node list

You should see nodes including ``/planner_server``, ``/controller_server``, and ``/bt_navigator``.

Confirm the planner is ready by checking for the ``/plan`` topic:

.. code-block:: bash

   ros2 topic list | grep plan
