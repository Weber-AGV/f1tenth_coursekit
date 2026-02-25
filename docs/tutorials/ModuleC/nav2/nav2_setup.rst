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

Before launching Nav2, the following must be ready:

- **A saved map** — you must have completed the SLAM tutorial and have ``lab_map.pgm`` and ``lab_map.yaml`` in ``~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/``
- **Terminal 1** — ``bringup`` (sensors + drivers)
- **RViz2** — open and ready to set the initial pose

Nav2 will launch its own map server internally using the map path you provide in the launch command — you do not need to run ``nav2_map_server`` separately.

.. seealso::

   Complete the :ref:`doc_setup_nav2` setup before proceeding. This installs the Nav2 packages and creates the F1TENTH parameters file.


Launch Nav2
-----------

Open a new terminal (Terminal 2):

.. code-block:: bash

   cd ~/f1tenth_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash

.. code-block:: bash

   ros2 launch nav2_bringup bringup_launch.py \
     use_sim_time:=False \
     map:=$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/lab_map.yaml \
     params_file:=$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/nav2_params.yaml

This launches the full Nav2 stack including the map server, planner, controller, and behavior tree navigator. The ``map`` argument tells Nav2's map server which map to load.

Verify Nav2 is Running
-----------------------

Check that the Nav2 lifecycle nodes are active:

.. code-block:: bash

   ros2 node list

You should see nodes including ``/map_server``, ``/planner_server``, ``/controller_server``, and ``/bt_navigator``.

Confirm the planner is ready by checking for the ``/plan`` topic:

.. code-block:: bash

   ros2 topic list | grep plan
