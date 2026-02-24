.. _doc_tutorials_waypoint_recording:

Recording Waypoints
====================

Pure Pursuit follows a pre-recorded path. Before the car can drive autonomously, you must drive it around the track manually once to record the waypoints.

How It Works
------------

While the particle filter is running and the car is localized, a waypoint logger node subscribes to ``/pf/pose/odom`` and saves each pose to a CSV file as you drive. The pure pursuit node later reads this CSV and follows the path.

Prerequisites
-------------

- ``bringup`` running (Terminal 1)
- ``ros2 launch particle_filter localize_launch.py`` running (Terminal 2)
- Initial pose set in RViz2 — car must be localized before recording

Steps
-----

1️⃣ Start the Waypoint Logger (Terminal 3)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   cd ~/f1tenth_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ros2 run pure_pursuit waypoint_logger.py

The logger will begin recording poses as the car moves.

2️⃣ Drive the Full Track
^^^^^^^^^^^^^^^^^^^^^^^^^

Using the PlayStation controller, drive the car around the complete track at a moderate speed. The logger saves a waypoint at each pose published by the particle filter.

**Tips for a good recording:**

- Drive at the speed you want the car to autonomously follow
- Cover the entire track — drive all sections at least once
- Close the loop — finish at approximately the same location you started
- Avoid sharp jerks or corrections, as these will be recorded into the path

3️⃣ Stop the Logger
^^^^^^^^^^^^^^^^^^^^^

When you have completed the full lap and returned near the start position, press **Ctrl+C** to stop the logger.

The waypoints are saved to:

.. code-block:: text

   ~/f1tenth_ws/src/pure_pursuit/maps/waypoints.csv

4️⃣ Verify the Recording
^^^^^^^^^^^^^^^^^^^^^^^^^

Check that the file was created and contains data:

.. code-block:: bash

   wc -l ~/f1tenth_ws/src/pure_pursuit/maps/waypoints.csv

A full lap should produce several hundred waypoints. If the count is very low, the car may not have been localized during recording — re-run with the particle filter active and initial pose set.

.. note::

   Once recorded, the waypoints file can be reused each session. You only need to re-record if the map changes or the track layout changes.
