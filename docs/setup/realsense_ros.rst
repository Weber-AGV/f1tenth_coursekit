.. _doc_setup_realsense_ros:


SETUP - Realsense ROS
==================================

For these instructions we will use the `realsense-ros <https://github.com/realsenseai/realsense-ros>`_ repository on GitHub.

Navigate to the f1tenth_ws

.. code-block:: bash

    cd ~/f1tenth_ws/src

Clone the realsense-ros repository

.. code-block:: bash

    git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
    cd ~/f1tenth_ws

Install dependencies:

.. code-block:: bash

    rosdep update 
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y

Build the workspace:

.. code-block:: bash

    colcon build --symlink-install

Source the workspace:

.. code-block:: bash

    source ~/f1tenth_ws/install/setup.bash

Start the Camera Node with Run or Launch


Run the realsense node:

.. code-block:: bash

    ros2 run realsense2_camera realsense2_camera_node
    # or, with parameters, for example - temporal and spatial filters are enabled:
    ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_color:=false -p spatial_filter.enable:=true -p temporal_filter.enable:=true

Launch the realsense node:

.. code-block:: bash

    ros2 launch realsense2_camera rs_launch.py
    ros2 launch realsense2_camera rs_launch.py depth_module.depth_profile:=1280x720x30 pointcloud.enable:=true

ROS2(Robot) vs Optical(Camera) Coordination Systems:

Point Of View:
- Imagine we are standing behind of the camera, and looking forward.
- Always use this point of view when talking about coordinates, left vs right IRs, position of sensor, etc..

.. image:: media/coordinate_camera.png
   :alt: realsense ros coordinate system
   :align: center
   :width: 400px

- ROS2 Coordinate System: (X: Forward, Y:Left, Z: Up)
- Camera Optical Coordinate System: (X: Right, Y: Down, Z: Forward)
- References: REP-0103, REP-0105
- All data published in our wrapper topics is optical data taken directly from our camera sensors.
- static and dynamic TF topics publish optical CS and ROS CS to give the user the ability to move from one CS to other CS.


Update the bringup_launch.py file

① Add these imports at the top:

.. code-block:: python

    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.conditions import IfCondition

② Add these launch arguments (so you can toggle it on/off):

.. code-block:: python

    realsense_enable_la = DeclareLaunchArgument(
        'realsense_enable',
        default_value='true',
        description='Launch RealSense D435i driver')

    realsense_depth_profile_la = DeclareLaunchArgument(
        'realsense_depth_profile',
        default_value='1280x720x30',
        description='RealSense depth profile (widthxheightxfps)')

    realsense_pointcloud_la = DeclareLaunchArgument(
        'realsense_pointcloud',
        default_value='true',
        description='Enable RealSense pointcloud')

③ Add these lines in the LaunchDescription() function:    

.. code-block:: python

    ld = LaunchDescription([joy_la, vesc_la, sensors_la, mux_la, realsense_enable_la, realsense_depth_profile_la, realsense_pointcloud_la])

