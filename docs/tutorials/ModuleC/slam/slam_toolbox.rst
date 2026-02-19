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

.. note::

   The ``slam`` command used below is an alias defined in your ``~/.bashrc`` file.
   This alias launches SLAM Toolbox using your robot’s configuration file.

Example (for robot ``f1-wsu-4``):

.. code-block:: bash

   alias slam='ros2 launch slam_toolbox online_async_launch.py \
   slam_params_file:=/home/f1-wsu-4/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml'

Replace ``f1-wsu-4`` with **your robot’s hostname**.

After adding the alias to ``~/.bashrc``, run:

.. code-block:: bash

   source ~/.bashrc


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


Fix: CUDA / range_libc Build Error
------------------------------------

If you encounter a CUDA compilation error when building ``range_libc`` on a Jetson Orin, use the following fix script.

1️⃣ Create the Script
^^^^^^^^^^^^^^^^^^^^^

Create the file ``~/f1tenth_ws/src/range_libc/fix_range_libc.sh``:

.. code-block:: bash

   nano ~/f1tenth_ws/src/range_libc/fix_range_libc.sh

Paste in the following script:

.. code-block:: bash

   #!/bin/bash
   # Fix and build range_libc with CUDA for Jetson Orin (sm_87)
   # Run from: ~/f1tenth_ws/src/range_libc/pywrapper/

   set -e

   if [ ! -f "setup.py" ]; then
       echo "ERROR: Run this script from range_libc/pywrapper/"
       echo "  cd ~/f1tenth_ws/src/range_libc/pywrapper && bash fix_range_libc.sh"
       exit 1
   fi

   echo "Patching setup.py..."

   # 1. Remove -march=native (nvcc doesn't support it)
   sed -i 's/"-march=native", //' setup.py

   # 2. Remove bogus gcc-8 flags (gcc-8 is not installed on these systems)
   sed -i '/gcc-8\|g++-8/d' setup.py

   # 3. Update CUDA arch from sm_62 (TX2) to sm_87 (Orin NX / Orin Nano)
   sed -i 's/-arch=sm_62/-arch=sm_87/' setup.py

   # 4. Fix _compile so non-.cu files are explicitly routed to gcc (not nvcc)
   python3 - <<'PYEOF'
   with open('setup.py', 'r') as f:
       content = f.read()

   old = (
       "        else:\n"
       "            postargs = extra_postargs['gcc']\n"
       "        # postargs = extra_postargs#['gcc']\n"
       "\n"
       "        super(obj, src, ext, cc_args, postargs, pp_opts)\n"
       "        # reset the default compiler_so, which we might have changed for cuda\n"
       "        self.compiler_so = default_compiler_so"
   )
   new = (
       "        else:\n"
       "            # explicitly reset to gcc for all non-.cu files\n"
       "            self.set_executable('compiler_so', default_compiler_so)\n"
       "            postargs = extra_postargs['gcc']\n"
       "\n"
       "        super(obj, src, ext, cc_args, postargs, pp_opts)\n"
       "        # reset the default compiler_so, which we might have changed for cuda\n"
       "        self.set_executable('compiler_so', default_compiler_so)"
   )

   if old in content:
       content = content.replace(old, new)
       with open('setup.py', 'w') as f:
           f.write(content)
       print("  [OK] Fixed _compile compiler routing")
   elif "set_executable('compiler_so', default_compiler_so)" in content:
       print("  [SKIP] _compile routing already fixed")
   else:
       print("  [WARN] Could not match _compile pattern - check setup.py manually")
   PYEOF

   echo "Building with CUDA (this takes a minute)..."
   sudo rm -rf build/
   sudo env WITH_CUDA=ON python3 setup.py install

   echo "Done! range_libc installed with CUDA support."

2️⃣ Make It Executable
^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   chmod +x ~/f1tenth_ws/src/range_libc/fix_range_libc.sh

3️⃣ Run the Script
^^^^^^^^^^^^^^^^^^

Navigate to the ``pywrapper`` directory first, then run the script:

.. code-block:: bash

   cd ~/f1tenth_ws/src/range_libc/pywrapper
   bash ../fix_range_libc.sh

.. note::

   The script must be run from inside the ``pywrapper/`` directory — it checks for ``setup.py`` in the current folder and will exit with an error if not found.
