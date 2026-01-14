.. _doc_tutorials_turtlesim_install:

Turtlesim Installation
======================

This guide walks you through installing and setting up turtlesim, a lightweight simulator for learning ROS 2 concepts.

1️⃣ Overview
~~~~~~~~~~~

Turtlesim is a simple 2D simulator that comes with ROS 2. It provides a visual tool for learning ROS 2 concepts such as nodes, topics, services, and actions. The turtle can be controlled through various ROS 2 commands and is perfect for understanding the basics of robot control.

2️⃣ Installation
~~~~~~~~~~~~~~~

Install turtlesim using apt
^^^^^^^^^^^^^^^^^^^^^^^^^^^

For ROS 2 Humble (or your specific ROS 2 distribution), install turtlesim with the following command:

.. code-block:: bash

   sudo apt update
   sudo apt install ros-humble-turtlesim

.. note::

   Replace ``humble`` with your ROS 2 distribution name if you're using a different version (e.g., ``foxy``, ``galactic``).

Verify Installation
^^^^^^^^^^^^^^^^^^^

After installation, verify that turtlesim is available:

.. code-block:: bash

   ros2 pkg list | grep turtlesim

You should see ``turtlesim`` listed in the output.

3️⃣ Running Turtlesim
~~~~~~~~~~~~~~~~~~~~

Launch the Turtlesim Node
^^^^^^^^^^^^^^^^^^^^^^^^^

To start the turtlesim simulator, run:

.. code-block:: bash

   ros2 run turtlesim turtlesim_node

This will open a window with a turtle in the center.

Control the Turtle
^^^^^^^^^^^^^^^^^^

In a new terminal, you can control the turtle using the keyboard teleop node:

.. code-block:: bash

   ros2 run turtlesim turtle_teleop_key

Use the arrow keys to move the turtle around the window.

4️⃣ Understanding Turtlesim Topics
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

List Active Topics
^^^^^^^^^^^^^^^^^^

While turtlesim is running, list the active topics:

.. code-block:: bash

   ros2 topic list

You should see topics like:

- ``/turtle1/cmd_vel``: Velocity commands to control the turtle
- ``/turtle1/pose``: Current position and orientation of the turtle
- ``/turtle1/color_sensor``: Color information from the turtle's location

Echo Topic Data
^^^^^^^^^^^^^^^

To see the data being published on a topic, use:

.. code-block:: bash

   ros2 topic echo /turtle1/pose

This will display the turtle's current position and orientation in real-time.

5️⃣ Next Steps
~~~~~~~~~~~~~

Now that turtlesim is installed and running, you can:

- Create custom nodes to control the turtle
- Experiment with ROS 2 services to spawn new turtles
- Use ROS 2 actions to make the turtle follow specific trajectories
- Visualize the ROS 2 graph using ``rqt_graph``

For more details on creating a controller for turtlesim, refer to the ROS 2 tutorial in this module.
