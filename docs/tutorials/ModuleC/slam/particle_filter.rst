.. _doc_tutorials_particle_filter:

Particle Filter Localization
=============================

Once you have a saved map, you can use a **particle filter** (Monte Carlo Localization) to localize the vehicle within that map in real time. This tutorial uses the `f1tenth/particle_filter <https://github.com/f1tenth/particle_filter>`_ package with **RangeLibc** for fast ray casting.

.. note::

   **RangeLibc** is a compiled Python library — it is imported automatically by the particle filter node at startup. You do not need to run it separately. As long as it was built correctly (see :ref:`doc_tutorials_slamtoolbox` for the fix script), it will work transparently in the background.

How It Works
------------

The particle filter maintains a set of hypotheses (particles) about where the car might be on the map. Each particle represents a possible pose (x, y, heading). On each LiDAR scan, the filter:

1. **Predicts** new particle positions using wheel odometry.
2. **Weights** each particle by how well its simulated LiDAR scan matches the real scan.
3. **Resamples** — particles that match the scan better survive; poor matches are discarded.
4. **Publishes** the best estimated pose.

Over time the particles converge on the vehicle's true location.

Topics
------

**Subscribed**

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - Topic
     - Type
     - Description
   * - ``/scan``
     - ``sensor_msgs/LaserScan``
     - LiDAR scan input
   * - ``/vesc/odom``
     - ``nav_msgs/Odometry``
     - Wheel odometry input
   * - ``/initialpose``
     - ``geometry_msgs/PoseWithCovarianceStamped``
     - Initial pose set via RViz2 "2D Pose Estimate"

**Published**

.. list-table::
   :header-rows: 1
   :widths: 30 30 40

   * - Topic
     - Type
     - Description
   * - ``/pf/pose/odom``
     - ``nav_msgs/Odometry``
     - Primary localized pose output
   * - ``/pf/viz/inferred_pose``
     - ``geometry_msgs/PoseStamped``
     - Best estimated pose for visualization
   * - ``/pf/viz/particles``
     - ``geometry_msgs/PoseArray``
     - All particle poses for visualization

Run Steps
---------

1️⃣ Point to Your Map
^^^^^^^^^^^^^^^^^^^^^^

Edit the map path in the launch file to point to your saved map:

.. code-block:: bash

   nano ~/f1tenth_ws/src/particle_filter/launch/localize.launch

Find the ``map_server.launch`` include and update the map argument to your map file:

.. code-block:: xml

   <arg name="map" default="$(find particle_filter)/maps/lab_map.yaml"/>

2️⃣ Bringup (Terminal 1)
^^^^^^^^^^^^^^^^^^^^^^^^^

Start the car stack as usual:

.. code-block:: bash

   cd ~/f1tenth_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   bringup

3️⃣ Launch the Particle Filter (Terminal 2)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   cd ~/f1tenth_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 launch particle_filter localize.launch

4️⃣ Open RViz2 (Terminal 3)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   source /opt/ros/humble/setup.bash
   rviz2

In RViz2:

- Set **Fixed Frame** to ``map``
- Add a **Map** display → Topic: ``/map``
- Add a **PoseArray** display → Topic: ``/pf/viz/particles``
- Add a **Pose** display → Topic: ``/pf/viz/inferred_pose``

5️⃣ Set the Initial Pose
^^^^^^^^^^^^^^^^^^^^^^^^^

The particle filter starts with particles spread randomly across the map. You must tell it roughly where the car is to begin localization:

- In RViz2, click **2D Pose Estimate** in the toolbar
- Click on the map where the car is located
- Drag in the direction the car is facing, then release

The particles will converge around your selected location.

.. note::

   The more accurately you set the initial pose, the faster the filter will converge. If the particles do not converge, try setting the pose again from a clearer location on the map.

6️⃣ Drive to Localize
^^^^^^^^^^^^^^^^^^^^^^

Drive the car using the PlayStation controller. As the car moves, the particle cloud will tighten and track the vehicle's position on the map. The best estimated pose is published on ``/pf/pose/odom``.

Key Parameters
--------------

You can tune the following parameters in ``launch/localize.launch``:

.. list-table::
   :header-rows: 1
   :widths: 25 15 60

   * - Parameter
     - Default
     - Description
   * - ``max_particles``
     - ``4000``
     - Number of particles — more particles = more accuracy but higher CPU cost
   * - ``angle_step``
     - ``18``
     - LiDAR downsample factor — higher = faster but less accurate
   * - ``range_method``
     - ``rmgpu``
     - Ray casting method (see below)
   * - ``motion_dispersion_theta``
     - ``0.25``
     - Heading noise — increase if filter diverges at speed

``range_method`` Options
~~~~~~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 20 20 60

   * - Method
     - Performance
     - Notes
   * - ``rmgpu``
     - Fastest
     - Requires RangeLibc compiled with CUDA — use on Jetson Orin
   * - ``cddt``
     - Fast
     - CPU only — recommended fallback if no CUDA
   * - ``pcddt``
     - Fast
     - Pruned variant of cddt
   * - ``rm``
     - Moderate
     - CPU only, no precomputation
   * - ``bl``
     - Slowest
     - Brute force — use only for debugging

.. note::

   If you built RangeLibc using the ``fix_range_libc.sh`` script on the Jetson Orin, ``rmgpu`` should work. If you see CUDA errors at runtime, fall back to ``cddt`` by editing ``range_method`` in the launch file and setting ``rangelib_variant`` to ``4``.

Troubleshooting
---------------

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Problem
     - Solution
   * - Particles spread across the whole map
     - Set the initial pose again using 2D Pose Estimate in RViz2
   * - Filter diverges while driving
     - Increase ``motion_dispersion_theta``; slow down; re-set pose
   * - CUDA errors on launch
     - Change ``range_method`` to ``cddt`` and ``rangelib_variant`` to ``4``
   * - ``/map`` not available
     - Ensure map_server is running and the map path is correct
   * - Particles not moving
     - Confirm ``/vesc/odom`` is publishing; check odometry topic name
