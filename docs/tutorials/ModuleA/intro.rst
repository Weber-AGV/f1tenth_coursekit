.. _doc_tutorials_intro:







Tutorial 1 - ROS 2
=============================

**Slide:** `Open in Google Slides <https://docs.google.com/presentation/d/1nzzvHQuBJAqdjlPDUl0l2_rU-MAu8jctu1Gt66vlHHM/edit?slide=id.g10838dd9226_0_0>`_

.. raw:: html

   <div style="max-width:100%; text-align:center; margin:0 auto;">
     <div style="position:relative; width:100%; padding-top:56.25%;">
       <iframe src="https://docs.google.com/presentation/d/1nzzvHQuBJAqdjlPDUl0l2_rU-MAu8jctu1Gt66vlHHM/embed?start=false&loop=false&delayms=3000&slide=id.g10838dd9226_0_0" style="position:absolute; top:0; left:0; width:100%; height:100%; border:0;" allowfullscreen="true" mozallowfullscreen="true" webkitallowfullscreen="true"></iframe>
     </div>
   </div>

You can ignore the docker section in this video

.. raw:: html

   <div style="max-width:100%; text-align:center; margin-top:1em;">
     <div style="position:relative; width:100%; padding-top:56.25%;">
       <iframe 
         src="https://www.youtube.com/embed/EU-QaO6xTv4?start=1300"
         title="YouTube video player"
         style="position:absolute; top:0; left:0; width:100%; height:100%; border:0;"
         frameborder="0"
         allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share"
         allowfullscreen>
       </iframe>
     </div>
   </div>





Create ``turtlesim_ws`` Workspace ROS 2
------------------------------------------

The following steps guide you to create a ROS 2 workspace named ``turtlesim_ws`` and include the ``turtlesim`` package for running a simple simulation.

---

1. Set Up the Workspace Directory
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Open a terminal and create the workspace directory structure:

.. code-block:: bash

   mkdir -p ~/turtlesim_ws/src
   cd ~/turtlesim_ws


2. Add the ``turtlesim`` Package
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``turtlesim`` package is part of the ROS 2 distribution and can be cloned or installed directly.

Option 1: Install Using ``apt``
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

For most ROS 2 distributions, you can install ``turtlesim``:

.. code-block:: bash

   sudo apt update
   sudo apt install ros-humble-turtlesim

This installs the turtlesim package system-wide on your machine.

.. note::

   No change to the structure of the ``turtlesim_ws`` workspace

.. image:: img/turtlesim_installation.png
   :alt: turtlesim_ws


3. Build the Workspace
~~~~~~~~~~~~~~~~~~~~~

Navigate to the workspace root directory and build the packages:

.. code-block:: bash

   cd ~/turtlesim_ws
   colcon build

.. image:: img/colcon_build.png
   :alt: colcon build

What Does ``colcon build`` Do?
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``colcon build`` command is used to build all the packages in a ROS 2 workspace. It automates the build process by managing dependencies, generating build artifacts, and installing outputs. Here's a breakdown of what happens during the ``colcon build`` process:

---

1. Initialize the Build Process
``````````````````````````````

- **Workspace Detection**: Colcon identifies the workspace by finding the ``src/`` directory containing ROS 2 packages.
- **Package Discovery**: Colcon scans the ``src/`` directory to locate all ROS 2 packages (defined by ``package.xml`` files).

---

2. Dependency Analysis
````````````````````

- **Dependency Graph**: Colcon examines the dependencies of each package based on its ``package.xml`` file and builds a dependency graph.
- **Build Order**: Packages are built in an order that satisfies their dependencies. For example, if Package A depends on Package B, Package B will be built first.

---

3. Invoke Build Systems
```````````````````````

- **Build Tool Selection**: Colcon identifies the build system used by each package (e.g., CMake, Python setuptools).

  - **CMake Packages**: Runs ``cmake`` to configure the package, followed by ``make`` or equivalent to compile it.
  - **Python Packages**: Uses ``setuptools`` to build Python packages.

- **Parallel Builds**: By default, Colcon builds packages in parallel when there are no dependency conflicts.

---

4. Generate Build Artifacts
```````````````````````````

- ``build/`` directory: Contains intermediate build files such as compiled binaries, object files, and configuration data.
- ``install/`` directory: Stores the final build outputs (e.g., executables, shared libraries, Python modules, and ROS message/service definitions).
- ``log/`` directory: Captures logs for each step of the build process, useful for debugging build issues.

---


5. Environment Setup
```````````````````

Colcon generates the necessary setup files (``setup.bash``, ``setup.zsh``, ``local_setup.bash``) in the ``install/`` directory.

These files allow your workspace's packages to be sourced and used in the ROS 2 environment.

---

6. Post-Build Steps
``````````````````

- Colcon validates the build process to ensure all steps completed successfully.
- If a package fails to build, Colcon provides logs and skips dependent packages (to prevent cascading failures).

---

Key Features of ``colcon build``
````````````````````````````````

1. **Modular Build**: Each package is built independently, reducing the impact of build failures.

2. **Incremental Builds**: Only packages that have been modified or whose dependencies have changed are rebuilt.

3. **Build Profiles**: Supports different profiles (e.g., ``Debug``, ``Release``) for optimized or debug builds.

---

Common Command Options
````````````````````

 
- ``--packages-select <package_name>``: Build only specified packages.
- ``--packages-ignore <package_name>``: Skip specific packages during the build.
- ``--parallel-workers <N>``: Control the number of parallel build jobs.
- ``--event-handlers console_cohesion+``: Improve the console output format for better readability.

---

By using ``colcon build``, you automate the process of compiling and installing your ROS 2 packages, ensuring that all dependencies are correctly resolved and that the workspace is ready for use.


4. Source the Workspace
~~~~~~~~~~~~~~~~~~~~~~

After the build is complete, source the workspace to use the installed packages:

.. note::

   We don't currently have a package for this workspace yet. We will be creating one shortly.

.. code-block:: bash

   source ~/turtlesim_ws/install/setup.bash

To make this permanent, add it to your ``.bashrc``:

.. code-block:: bash

   echo "source ~/turtlesim_ws/install/setup.bash" >> ~/.bashrc
   source ~/.bashrc


Summary of What a Workspace Is in ROS 2
----------------------------------------

A ROS 2 workspace is a directory structure used to organize and manage ROS 2 packages. It serves as the development environment where you store, build, and work with packages that are part of your ROS 2 project. A workspace includes several directories and components, each with a specific purpose.



.. centered:: Create and Setup a Package


What Is a ROS 2 Package?
~~~~~~~~~~~~~~~~~~~~~~~~~

A **ROS 2 package** is a modular unit of functionality in ROS 2 that contains everything needed to perform a specific task or provide a feature. Packages are the fundamental building blocks of ROS 2 applications and can include nodes, libraries, configuration files, launch files, and other resources.

Key Components of a ROS 2 Package
---------------------------------

1. ``package.xml``:
   - Metadata file that describes the package (name, version, description, dependencies).
   - Used by build tools to resolve dependencies.

2. ``CMakeLists.txt``:
   - Build configuration file for C++ or other compiled packages.
   - Specifies how the package should be built.

3. ``setup.py``:
   - Python build script for Python-based packages.

4. Source Code:
   - Python scripts, C++ source files, or libraries implementing the package functionality.
   - Typically placed in a directory named after the package.

5. Resources:
   - Configuration files, message/service/action definitions, launch files, etc., necessary for the package's operation.

What Makes a Package Special in ROS 2?
--------------------------------------

1. **Reusability**:
   - Packages are self-contained, making it easy to share and reuse them across different projects.

2. **Dependency Management**:
   - Packages declare their dependencies in ``package.xml``, allowing tools like ``colcon`` to ensure everything needed is installed and built.

3. **Interoperability**:
   - ROS 2 packages can communicate with each other using topics, services, and actions, enabling modular system design.

4. **Scalability**:
   - Complex applications can be developed by combining multiple smaller, focused packages.

How a Package Fits Into a Workspace
-----------------------------------

- A package resides in the ``src/`` directory of a ROS 2 workspace.
- It is built and installed into the workspace's ``build/`` and ``install/`` directories.
- After sourcing the workspace, the package's nodes, launch files, and resources become available for use.

Example Use Case
----------------

If you are controlling a robot, you might have:
- A **sensor package** to process sensor data.
- A **control package** to compute actions.
- A **simulation package** to visualize the robot's behavior.

Each package would handle one aspect of the overall application.

In essence, a ROS 2 package is a self-contained module that organizes code, dependencies, and resources for a specific purpose, allowing you to build scalable and modular robotic applications.

1. Navigate to Your Workspace
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Ensure you are in the workspace where you want to create the package:

.. code-block:: bash

   cd ~/turtlesim_ws/src


2. Create the Package
~~~~~~~~~~~~~~~~~~~

Use the ``ros2 pkg create`` command to create the ``my_turtlesim_controller`` package:

.. image:: img/ros2_pkg_create.png
   :alt: ros2 pkg create


.. code-block:: bash

   ros2 pkg create my_turtlesim_controller --build-type ament_python --dependencies rclpy


.. image:: img/pkg_create.png
   :alt: pkg create


3. Understand the Generated Files
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The command creates the following structure::

   my_turtlesim_controller/
   ├── CMakeLists.txt          # CMake build file (not used for Python packages)
   ├── package.xml             # Defines package metadata and dependencies
   ├── resource/               # Stores package-specific resources
   │   └── my_turtlesim_controller  # Empty marker file for ament resource index
   ├── setup.py                # Python build script
   └── my_turtlesim_controller/ # Directory for Python scripts

Open up the src in Visual Studio Code to view:

.. code-block:: bash

   cd ~/turtlesim_ws/src
   code .


.. image:: img/code_structure.png
   :alt: file structure


4. Add Dependencies
~~~~~~~~~~~~~~~~~

Edit ``package.xml`` to declare the package dependencies. For controlling turtlesim, include ``rclpy`` and ``turtlesim``:

.. code-block:: xml

   <depend>turtlesim</depend>


.. image:: img/depend.png
   :alt: package.xml dependency

Add your email to the maintainer email line in ``package.xml``:

.. image:: img/email.png
   :alt: maintainer email


5. Colcon build
~~~~~~~~~~~~~~

Navigate back to the turtlesim_ws directory:

.. code-block:: bash

   cd ~/turtlesim_ws


Use colcon build to build the new package. Note and explanation:

.. note::

   The `colcon build` command:

   - Identifies the packages in the ``src/`` directory based on the presence of ``package.xml`` files.
   - Resolves dependencies, determines build order, and invokes the appropriate build system (e.g., CMake for C++ or setuptools for Python).

.. code-block:: bash

   colcon build

If you get the setup tools error, follow the instructions below:

.. image:: img/error_setup_tools.png
   :alt: setup tools error

.. code-block:: bash

   pip3 install setuptools==58.2.0


6. Write the Python Code
~~~~~~~~~~~~~~~~~~~~~~~

Create a Python script to control the turtle. In the terminal navigate to the package's python directory:

.. code-block:: bash

   cd ~/turtlesim_ws/src/my_turtlesim_controller/my_turtlesim_controller

Create a file called ``node_turtle_controller.py``:

.. code-block:: bash

   touch node_turtle_controller.py

Make the file executable:

.. code-block:: bash

   chmod +x node_turtle_controller.py

Go back to src and open it in vscode:

.. code-block:: bash

   cd ../..
   code .

Install the ROS extension in vscode:

.. image:: img/ros_extension.png
   :alt: ros extension


This is an example Python node (save as ``node_turtle_controller.py``):

.. code-block:: python

   #!/usr/bin/env python3
   import rclpy  # Import the ROS 2 Python client library
   from rclpy.node import Node  # Import the Node base class for creating ROS 2 nodes
   from geometry_msgs.msg import Twist  # Import the Twist message type for velocity commands

   class TurtleController(Node):
       """
       A ROS 2 Node to control the turtle in turtlesim by publishing velocity commands.
       """
       def __init__(self):
           # Initialize the Node with the name 'turtle_controller'
           super().__init__('turtle_controller')
           
           # Create a publisher to the '/turtle1/cmd_vel' topic
           self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
           
           # set up a timer that calls the move_turtle method every 0.5 seconds
           self.timer = self.create_timer(0.5, self.move_turtle)
           
           # Log a message indicating the node has been started
           self.get_logger().info('Turtle Controller Node has started.')

       def move_turtle(self):
           """
           Publishes a velocity command to make the turtle move.
           """
           twist = Twist()
           twist.linear.x = 2.0
           twist.angular.z = 1.0
           self.publisher.publish(twist)
           self.get_logger().info('Published velocity command.')

   def main(args=None):
       rclpy.init(args=args)
       node = TurtleController()
       try:
           node.move_turtle()
           rclpy.spin(node)
       except KeyboardInterrupt:
           pass
       finally:
           node.destroy_node()
           rclpy.shutdown()

   if __name__ == '__main__':
       main()


7. Update ``setup.py``
~~~~~~~~~~~~~~~~~~~

Update the ``setup.py`` file to add an entry point so the script is runnable as a console script:

.. code-block:: python

   entry_points={
       'console_scripts': [
           'turtle_controller = my_turtlesim_controller.node_turtle_controller:main',
       ],
   },


8. Build the Package
~~~~~~~~~~~~~~~~~~~

Navigate back to the root of the workspace and build your package:

.. code-block:: bash

   cd ~/turtlesim_ws
   colcon build --packages-select my_turtlesim_controller

.. note::

   If you use ``--symlink-install`` when developing Python packages, you won't need to re-run ``colcon build`` after every code change.


9. Source the Workspace
~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   source ~/turtlesim_ws/install/setup.bash

.. note::

   If you added the source line to your ``.bashrc``, you can instead run ``source ~/.bashrc`` to load the workspace.


10. Test the `my_turtlesim_controller` Node
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

.. code-block:: bash

   ros2 run turtlesim turtlesim_node

In another terminal:

.. code-block:: bash

   ros2 run my_turtlesim_controller turtle_controller

In another terminal:

.. code-block:: bash

   ros2 topic echo /turtle1/cmd_vel

In another terminal:

.. code-block:: bash

   rqt_graph

The turtle should move in a circular motion based on the linear and angular velocities you defined.




