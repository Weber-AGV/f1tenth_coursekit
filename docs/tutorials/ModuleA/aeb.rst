.. _doc_tutorials_aeb:

Tutorial 2 - Automatic Emergency Braking (AEB)
=============================

1️⃣ Learning Goals
~~~~~~~~~~~~~~~~~

- Using the ``LaserScan`` message in ROS 2
- Instantaneous Time to Collision (iTTC)
- Safety critical systems

2️⃣ Overview
~~~~~~~~~~~

The goal of this lab is to develop a safety node for the race cars that will stop the car from collision when travelling at higher velocities. We will implement Instantaneous Time to Collision (iTTC) using the ``LaserScan`` message in the simulator.

For different commonly used ROS 2 messages you can use ``ros2 interface show <msg_name>`` to see the definition of messages. Note for messages that are not installed by default by the distro, you'll have to first install it for this to work.

The ``LaserScan`` Message
^^^^^^^^^^^^^^^^^^^^^^^^^

The `LaserScan <http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html>`_ message contains several fields that will be useful to us. You can see detailed descriptions of what each field contains in the API. The one we'll be using the most is the ``ranges`` field. This is an array that contains all range measurements from the LiDAR radially ordered. You'll need to subscribe to the ``/scan`` topic and calculate iTTC with the LaserScan messages.

The ``Odometry`` Message
^^^^^^^^^^^^^^^^^^^^^^^^

Both the simulator node and the car itself publish `Odometry <http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html>`_ messages. Within its several fields, the message includes the cars position, orientation, and velocity. You'll need to explore this message type in this lab.

The ``AckermannDriveStamped`` Message
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You've already used `AckermannDriveStamped <http://docs.ros.org/en/jade/api/ackermann_msgs/html/msg/AckermannDriveStamped.html>`_ in the previous lab. It will be the message type that we'll use throughout the course to send driving commands to the simulator and the car. In the simulator, you can stop the car by sending an ``AckermannDriveStamped`` message with the ``speed`` field set to 0.0.


3️⃣ The TTC Calculation
~~~~~~~~~~~~~~~~~~~~~~

Time to Collision (TTC) is the time it would take for the car to collide with an obstacle if it maintained its current heading and velocity. We approximate the time to collision using Instantaneous Time to Collision (iTTC), which is the ratio of instantaneous range to range rate calculated from current range measurements and velocity measurements of the vehicle.

As discussed in the lecture, we can calculate the iTTC as:

.. math::

   iTTC = \frac{r}{\{-\dot{r}\}_{+}}

where $r$ is the instantaneous range measurements, and $\dot{r}$ is the current range rate for that measurement.
And the operator $\{x\}_{+} = \max(x, 0)$.

The instantaneous range $r$ to an obstacle is easily obtained by using the current measurements from the ``LaserScan`` message. Since the LiDAR effectively measures the distance from the sensor to some obstacle.

The range rate $\dot{r}$ is the expected rate of change along each scan beam. A positive range rate means the range measurement is expanding, and a negative one means the range measurement is shrinking.

Thus, it can be calculated in two different ways:

1. **Using Velocity Projection**: Calculate by mapping the vehicle's current longitudinal velocity onto each scan beam's angle by using $v_x \cos{\theta_{i}}$. Be careful with assigning the range rate a positive or a negative value. The angles could also be determined by the information in ``LaserScan`` messages.

2. **Using Range Difference**: Take the difference between the previous range measurement and the current one, divide it by how much time has passed in between (timestamps are available in message headers), and calculate the range rate that way.

Note the negation in the calculation — this is to correctly interpret whether the range measurement should be decreasing or increasing. For a vehicle travelling forward towards an obstacle, the corresponding range rate for the beam right in front of the vehicle should be negative since the range measurement should be shrinking. Vice versa, the range rate corresponding to the vehicle travelling away from an obstacle should be positive since the range measurement should be increasing. The operator is in place so the iTTC calculation will be meaningful. When the range rate is positive, the operator will make sure iTTC for that angle goes to infinity.

After your calculations, you should end up with an array of iTTCs that correspond to each angle. When a time to collision drops below a certain threshold, it means a collision is imminent.


4️⃣ Automatic Emergency Braking with iTTC
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For this lab, you will make a Safety Node that should halt the car before it collides with obstacles. To do this, you will make a ROS 2 node that subscribes to the ``LaserScan`` and ``Odometry`` messages. It should analyze the ``LaserScan`` data and, if necessary, publish an ``AckermannDriveStamped`` with the ``speed`` field set to 0.0 m/s to brake.

After you've calculated the array of iTTCs, you should decide how to proceed with this information. You'll have to decide how to threshold, and how to best remove false positives (braking when collision isn't imminent). Don't forget to deal with ``inf``s or ``nan``s in your arrays.

**Topic Names**:

- ``LaserScan``: ``/scan``
- ``Odometry``: ``/odom`` (specifically, the longitudinal velocity can be found in ``twist.twist.linear.x``)
- ``AckermannDriveStamped``: ``/drive``


5️⃣ Deliverables and Submission
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can implement this node in either C++ or Python. A skeleton package is available on the AEB Notes Page. Put your package in ``/src`` folder.

**Deliverable 1**: After you're finished, update the entire skeleton package directory with your ``safety_node``. Upload your code to canvas.

**Deliverable 2**: Make a screen cast of running your safety node. Drive the car showing it doesn't brake when traveling straight in the hallway. You need to show that your safe node doesn't generate false positives (i.e., the car doesn't suddenly stop while traveling down the hallway). Then show the car driving towards an object and braking correctly. Upload your video to canvas.


6️⃣ Grading Rubric
~~~~~~~~~~~~~~~~~

- **Compilation**: 30 points
- **Provided Video**: 20 points
- **Correctly stops before collision**: 30 points
- **Correctly calculates TTC**: 10 points
- **Able to navigate through the hallway**: 10 points


AEB Notes
~~~~~~~~~

To set up your safety package within the driver stack container and prevent the roboracer from colliding with objects in front of it, follow these steps.

1️⃣ Create the Safety Package
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Inside the driver stack container, navigate to your ROS 2 workspace ``~/f1tenth_ws/src`` and create a new package:

.. code-block:: bash

   cd ~/f1tenth_ws/src
   ros2 pkg create safety_package --build-type ament_python --dependencies rclpy sensor_msgs std_msgs

.. note::

   **Dependencies**:

   - ``rclpy``: ROS Client Library for Python
   - ``sensor_msgs``: Standard Messages for Sensors


2️⃣ Modify package.xml
^^^^^^^^^^^^^^^^^^^^^

Ensure ``package.xml`` includes dependencies like ``rclpy``, ``sensor_msgs``, and ``std_msgs``. Open ``package.xml`` and add:

.. code-block:: xml

   <depend>rclpy</depend>
   <depend>sensor_msgs</depend>
   <depend>std_msgs</depend>

If you're using C++ instead of Python, also ensure you have:

.. code-block:: xml

   <depend>rclcpp</depend>
   <depend>tf2_ros</depend>
   <depend>geometry_msgs</depend>


3️⃣ Modify CMakeLists.txt (If Using C++)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you're using C++, modify ``CMakeLists.txt`` to include:

.. code-block:: cmake

   find_package(rclcpp REQUIRED)
   find_package(sensor_msgs REQUIRED)
   find_package(std_msgs REQUIRED)

Ensure the ``add_executable`` or ``ament_target_dependencies`` includes the necessary dependencies.


4️⃣ Install Dependencies Using rosdep
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Run the following to install missing dependencies:

.. code-block:: bash

   cd ~/f1tenth_ws
   rosdep install --from-paths src --ignore-src -r -y


5️⃣ Implement the Safety Node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You'll create a safety node that listens to LiDAR data (``/scan``) and publishes a safety brake command if an object is too close.

**Example: Python Safety Node (safety_node.py)**

Create a file inside ``safety_package/safety_node.py``:

.. code-block:: python

   #!/usr/bin/env python3
   import rclpy
   from rclpy.node import Node

   import numpy as np
   # TODO: include needed ROS msg type headers and libraries
   from sensor_msgs.msg import LaserScan
   from nav_msgs.msg import Odometry
   from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


   class SafetyNode(Node):
       """
       The class that handles emergency braking.
       """
       def __init__(self):
           super().__init__('safety_node')
           """
           One publisher should publish to the /drive topic with a AckermannDriveStamped drive message.

           You should also subscribe to the /scan topic to get the LaserScan messages and
           the /odom topic to get the current speed of the vehicle.

           The subscribers should use the provided odom_callback and scan_callback as callback methods

           NOTE that the x component of the linear velocity in odom is the speed
           """
           self.speed = 0.
           # TODO: create ROS subscribers and publishers.

       def odom_callback(self, odom_msg):
           # TODO: update current speed
           self.speed = 0.

       def scan_callback(self, scan_msg):
           # TODO: calculate TTC
           
           # TODO: publish command to brake
           pass

   def main(args=None):
       rclpy.init(args=args)
       safety_node = SafetyNode()
       rclpy.spin(safety_node)

       # Destroy the node explicitly
       # (optional - otherwise it will be done automatically
       # when the garbage collector destroys the node object)
       safety_node.destroy_node()
       rclpy.shutdown()


   if __name__ == '__main__':
       main()


6️⃣ Make It Executable
^^^^^^^^^^^^^^^^^^^^^

Modify ``setup.py`` inside ``safety_package``:

.. code-block:: python

   entry_points={
       'console_scripts': [
           'safety_node = safety_package.safety_node:main',
       ],
   },


7️⃣ Build & Run
^^^^^^^^^^^^^^

Run the following:

.. code-block:: bash

   cd ~/f1tenth_ws
   colcon build --packages-select safety_package
   source install/setup.bash
   ros2 run safety_package safety_node

This will listen to LiDAR (``/scan``) and publish a safety stop signal (``/safety_brake``) when an obstacle is too close.