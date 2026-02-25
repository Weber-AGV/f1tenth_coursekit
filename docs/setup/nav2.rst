.. _doc_setup_nav2:

SETUP - Nav2
==================================

Install Nav2 (Navigation Stack)
---------------------------------

1️⃣ **Install the Nav2 packages**

.. code-block:: console

   sudo apt update
   sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

``ros-humble-navigation2`` provides the core Nav2 nodes (planner, controller, bt_navigator) and ``ros-humble-nav2-bringup`` provides the launch files.


Create the Parameters File
---------------------------

Nav2's built-in defaults are tuned for a Turtlebot3 (differential drive, 0.26 m/s max speed, wrong robot radius). You need a parameters file tuned for the F1TENTH Ackermann platform.

2️⃣ **Create the file on the robot**

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

3️⃣ **Verify the file exists**

.. code-block:: bash

   ls ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/nav2_params.yaml
