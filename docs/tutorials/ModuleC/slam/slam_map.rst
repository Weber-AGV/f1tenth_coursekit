.. _doc_tutorials_slam_map:

Saving the Map
==============

Once you have finished building your map with SLAM Toolbox, you must save it for later use in localization or navigation.

🎥 Video Walkthrough
--------------------

.. raw:: html

   <iframe width="560" height="315"
     src="https://www.youtube.com/embed/RanGbHii2m8"
     title="YouTube video player"
     frameborder="0"
     allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
     allowfullscreen></iframe>

|

⚠️ Important
------------

- The car must remain powered on while saving.
- SLAM Toolbox must still be running.
- Do **not** close the SLAM terminal before saving the map.


Steps to Save the Map
---------------------

1️⃣ Finish Driving the Map  
^^^^^^^^^^^^^^^^^^^^^^^^^^

- Drive the vehicle through the entire area.
- Make sure the loop is closed if possible.
- Verify the map looks complete in RViz2.


2️⃣ Open a New Terminal  
^^^^^^^^^^^^^^^^^^^^^^^^

Open a new terminal on the RoboRacer:

.. code-block:: bash

   cd ~/f1tenth_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash


3️⃣ Save the Map
^^^^^^^^^^^^^^^^^

In ROS 2 Humble, use the ``nav2_map_server`` package to save the map. Save it directly into the stack's maps folder to avoid any extra move steps.

First, create the maps directory in the stack (if it does not already exist):

.. code-block:: bash

   mkdir -p ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps

Then run the map saver command, saving directly into that folder:

.. code-block:: bash

   ros2 run nav2_map_server map_saver_cli -f ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/lab_map

This will generate two files:

.. code-block:: text

   ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/lab_map.pgm
   ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/lab_map.yaml

- ``.pgm`` — grayscale occupancy grid image
- ``.yaml`` — metadata (resolution, origin, thresholds)


4️⃣ Verify the Map Was Saved
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Check that both files exist:

.. code-block:: bash

   ls ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/

You should see:

.. code-block:: text

   lab_map.pgm
   lab_map.yaml

Then open the yaml file and confirm the ``image:`` field matches the ``.pgm`` filename exactly:

.. code-block:: bash

   cat ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/lab_map.yaml

You should see a line like:

.. code-block:: text

   image: lab_map.pgm


5️⃣ Load the Map
^^^^^^^^^^^^^^^^^

In ROS 2 Humble, the map server is part of the ``nav2_map_server`` package and runs as a **lifecycle node**. You must start it in one terminal, then configure and activate it in a second terminal.

**Terminal 1** — Start the map server:

.. code-block:: bash

   ros2 run nav2_map_server map_server --ros-args \
     -p yaml_filename:=$HOME/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/lab_map.yaml

**Terminal 2** — Configure then activate:

.. code-block:: bash

   ros2 lifecycle set /map_server configure
   ros2 lifecycle set /map_server activate

The map will then be published on the ``/map`` topic.

.. note::

   The lifecycle transitions are required — the map server will not publish until it is both configured **and** activated. If ``configure`` fails (e.g. due to a bad yaml ``image:`` path), the node enters an error state and ``activate`` will not be available. Fix the yaml and restart Terminal 1 before trying again.


6️⃣ Verify the Map in RViz2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open a new terminal and launch RViz2:

.. code-block:: bash

   source /opt/ros/humble/setup.bash
   rviz2

In RViz2:

- Set **Fixed Frame** to ``map``
- Click **Add** → select **Map** → set Topic to ``/map``

You should see your saved map appear. This confirms the map server is running and publishing correctly.


Common Mistakes
---------------

- ❌ Closing SLAM before saving  
- ❌ Forgetting to source the workspace  
- ❌ Saving in the wrong directory  
- ❌ Not having a complete map before saving  

Next Step
---------

After saving, the map can be used for:

- Localization
- Autonomous navigation
- Path planning
