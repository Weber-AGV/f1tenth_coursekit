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

In ROS 2 Humble, use the nav2_map_server package to save the map.

First, create a maps directory in your workspace (if it does not already exist):

.. code-block:: bash

mkdir -p ~/f1tenth_ws/maps

Then run the map saver command and save directly into that folder:

.. code-block:: bash

cd ~/f1tenth_ws/maps
ros2 run nav2_map_server map_saver_cli -f lab_map

This will generate two files:

::

lab_map.pgm
lab_map.yaml

These files contain:

.pgm → grayscale occupancy grid image

.yaml → metadata (resolution, origin, thresholds)

Both files will now be stored in:

::

~/f1tenth_ws/maps


4️⃣ Move Map to the Maps Folder (Recommended)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

It is good practice to store maps in your stack's maps folder.

Example:

.. code-block:: bash

   mv my_map.* ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/


5️⃣ Load the Map
^^^^^^^^^^^^^^^^^

To load a saved map with the Map Server, run:

.. code-block:: bash

   ros2 run map_server map_server --ros-args -p yaml_filename:=my_map.yaml

.. note::

   Replace ``my_map.yaml`` with the full path to your map file if it is not in the current directory. For example:

   .. code-block:: bash

      ros2 run map_server map_server --ros-args -p yaml_filename:=~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/my_map.yaml


Verify Files
------------

Check that both files exist:

.. code-block:: bash

   ls ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/

You should see:

::

   my_map.pgm
   my_map.yaml


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
