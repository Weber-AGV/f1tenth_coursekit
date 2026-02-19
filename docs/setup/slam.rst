.. _doc_setup_slam:

SETUP - SLAM
==================================

Install SLAM Toolbox
--------------------

1️⃣ **Install the SLAM Toolbox package**

.. code-block:: console

   sudo apt update
   sudo apt install ros-humble-slam-toolbox

.. note::

   If you get an apt lock error mentioning ``packagekitd``, stop it and try again:

   .. code-block:: console

      sudo systemctl stop packagekit
      sudo systemctl disable packagekit
      sudo apt update
      sudo apt install -y ros-humble-slam-toolbox



2️⃣ **Add an alias to your `~/.bashrc` to launch SLAM Toolbox**

.. note::

   Modify the path below for the specific robot you are using.
   This example is for ``f1-wsu-4``.

.. code-block:: console

   alias slam='ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/f1-wsu-4/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml'

After editing ``~/.bashrc``:

.. code-block:: console

   source ~/.bashrc

Open a new terminal to verify the alias works.

Install pip (Required for rosdep)
----------------------------------

Some dependencies require ``pip`` to be installed.

1️⃣ Install pip

.. code-block:: console

   sudo apt update
   sudo apt install -y python3-pip

2️⃣ Verify pip installation

.. code-block:: console

   pip3 --version


Install Particle Filter (Localization)
---------------------------------------

1️⃣ **Clone the particle_filter package**

.. code-block:: console

   cd ~/f1tenth_ws/src
   git clone https://github.com/f1tenth/particle_filter.git


2️⃣ **Install workspace dependencies**

.. code-block:: console

   cd ~/f1tenth_ws
   rosdep install -r --from-paths src --ignore-src --rosdistro humble -y


3️⃣ **Rebuild the workspace**

.. code-block:: console

   cd ~/f1tenth_ws
   colcon build
   source install/setup.bash


Install range_libc
------------------

1️⃣ **Clone the repository**

.. code-block:: console

   cd ~/f1tenth_ws/src
   git clone https://github.com/f1tenth/range_libc.git


2️⃣ **Install Cython (required)**

.. code-block:: console

   sudo apt install -y python3-cython



3️⃣ **Install range_libc with CUDA support**

.. code-block:: console

   cd range_libc/pywrapper
   sudo WITH_CUDA=ON python3 setup.py install


Launch Localization
-------------------

1️⃣ **Launch teleop (in one terminal)**

.. code-block:: console

   bringup

2️⃣ **Launch particle filter (in another terminal)**

.. code-block:: console

   source ~/f1tenth_ws/install/setup.bash
   ros2 launch particle_filter localize_launch.py


Common Notes
------------

- Make sure you are using the correct ROS distribution (Humble).
- Always source your workspace after building.
- If CUDA errors occur, confirm your Jetson has CUDA installed and accessible.
