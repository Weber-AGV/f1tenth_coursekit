.. _doc_setup_slam:


SETUP - SLAM
==================================

Install SLAM Toolbox
--------------------

- 1️⃣ **Install the SLAM Toolbox package**

  .. code-block:: console

     sudo apt install ros-humble-slam-toolbox

- 2️⃣ **Add an alias to your `~/.bashrc` to launch SLAM Toolbox**

  .. note::

     Modify the path below for the specific robot that your are using. This is for f1-wsu-4

  .. code-block:: console

     alias slam='ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/f1-wsu-4/f1tenth_ws/src/f1tenth_system/f1tenth_stack/config/f1tenth_online_async.yaml'

  > **Tip:** After editing `~/.bashrc`, run `source ~/.bashrc` or open a new terminal to use the alias. Adjust the path if your workspace or username differs.


