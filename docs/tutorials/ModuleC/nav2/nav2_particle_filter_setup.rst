.. _doc_tutorials_nav2_particle_filter_setup:

Setup — Particle Filter (Updating Other Robots)
=================================================

If you have configured Nav2 and the particle filter on one robot, follow these steps to apply the same setup to your other robots. This moves the ``particle_filter`` and ``range_libc`` packages into the ``f1tenth_system`` directory so they are tracked by git.

.. warning::

   These steps delete the standalone ``particle_filter`` and ``range_libc`` directories. Make sure you have backed up any student maps before proceeding.

Step 1 — Back Up Student Maps
-------------------------------

Before making any changes, back up your saved maps:

.. code-block:: bash

   cp -r ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps ~/maps_backup
   ls ~/maps_backup

Step 2 — Pull the Latest nav2-devel
-------------------------------------

.. code-block:: bash

   cd ~/f1tenth_ws/src/f1tenth_system
   git fetch origin
   git checkout nav2-devel
   git pull origin nav2-devel

If there are merge conflicts in ``f1tenth_stack/maps/``, resolve by keeping both versions:

.. code-block:: bash

   git checkout --theirs f1tenth_stack/maps/   # take the remote version
   cp ~/maps_backup/*.pgm ~/maps_backup/*.yaml f1tenth_stack/maps/  # restore student maps on top
   git add f1tenth_stack/maps/
   git merge --continue

Step 3 — Initialize the New Submodules
----------------------------------------

.. code-block:: bash

   git submodule update --init --recursive

Step 4 — Remove the Old Standalone Directories
------------------------------------------------

.. code-block:: bash

   rm -rf ~/f1tenth_ws/src/particle_filter
   sudo rm -rf ~/f1tenth_ws/src/range_libc   # needs sudo — build artifacts are root-owned

Step 5 — Rebuild
-----------------

.. code-block:: bash

   cd ~/f1tenth_ws
   colcon build --symlink-install
   source install/setup.bash

Step 6 — Restore Any Student Maps
------------------------------------

Compare what is in your backup versus the current maps directory and copy anything missing:

.. code-block:: bash

   ls ~/maps_backup/
   ls ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/

Copy any missing student map files:

.. code-block:: bash

   # Replace STUDENT_MAP with your actual map name
   cp ~/maps_backup/STUDENT_MAP.pgm ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/
   cp ~/maps_backup/STUDENT_MAP.yaml ~/f1tenth_ws/src/f1tenth_system/f1tenth_stack/maps/

Step 7 — Rebuild range_libc
-----------------------------

The robots need to rebuild ``range_libc`` since the build artifacts from the old location are gone:

.. code-block:: bash

   cd ~/f1tenth_ws/src/f1tenth_system/range_libc/pywrapper
   sudo env WITH_CUDA=ON python3 setup.py install

.. note::

   The ``fix_range_libc.sh`` script is no longer needed since those fixes are now baked into the repo.
