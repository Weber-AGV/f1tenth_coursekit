.. _doc_tutorials_slamtoolbox:

SLAM Toolbox
===========================

This page shows how to run SLAM Toolbox on the RoboRacer and visualize the map in RViz2.

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

You must have the **controller powered on and connected** before starting SLAM.  
If the controller is not connected, the vehicle will not publish drive data and SLAM will not begin building a map.

Command Alias
-------------

We use an alias named ``slam`` that launches SLAM Toolbox with the F1TENTH config.

Replace ``nvidia`` with **your RoboRacer username / hostname**
(example: ``f1-wsu-1``).

.. code-block:: bash

   slam

What ``slam`` runs
^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 launch slam_toolbox online_async_launch.py \
     params_file:=/home/nvidia/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml

If your RoboRacer name is ``f1-wsu-1``, your path would look like:

.. code-block:: bash

   ros2 launch slam_toolbox online_async_launch.py \
     params_file:=/home/f1-wsu-1/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml


Run Steps
---------

1️⃣ Bringup (Terminal 1)
^^^^^^^^^^^^^^^^^^^^^^^^

Start the car stack (sensors + drivers + base nodes).

.. code-block:: bash

   cd ~/f1tenth_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash

   bringup


2️⃣ Turn On and Connect Controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Power on the transmitter
- Confirm it is bound to the vehicle
- Verify you have steering and throttle response

SLAM will not build a map unless the car is moving.


3️⃣ Start SLAM Toolbox (Terminal 2)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open a **separate terminal** and run:

.. code-block:: bash

   cd ~/f1tenth_ws
   source /opt/ros/humble/setup.bash
   source install/setup.bash

   slam


4️⃣ Open RViz2 (Terminal 3)
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Open another terminal (often on your laptop) and run:

.. code-block:: bash

   source /opt/ros/humble/setup.bash
   rviz2


Configure RViz2
---------------

After RViz2 opens:

1️⃣ Set the Fixed Frame  
^^^^^^^^^^^^^^^^^^^^^^^^

- Top left → **Global Options**
- Set **Fixed Frame** to:

::

   map


2️⃣ Add Map Display  
^^^^^^^^^^^^^^^^^^^^

- Click **Add**
- Select **Map**

Then configure the Map display:

- **Topic** → make sure it is:

::

   /map

- **Topic → History Policy** → ``Keep All``
- **Topic → Durability Policy** → ``Transient Local``
- **Update Topic → Durability Policy** → ``Transient Local``


3️⃣ Add GraphVisualization  
^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Click **Add**
- Select **GraphVisualization**


4️⃣ Add SLAM Toolbox Panel  
^^^^^^^^^^^^^^^^^^^^^^^^^^^

- Top left → **Panels**
- Select **Add New Panel**
- Choose:

::

   SlamToolBoxPlugin


Common Checks
-------------

- Confirm ``/scan`` is publishing.
- Confirm controller is connected.
- Confirm Fixed Frame is ``map``.
- Confirm Map topic is ``/map``.
- Confirm Durability is set to ``Transient Local``.
- Confirm both machines use the same ``ROS_DOMAIN_ID``.
