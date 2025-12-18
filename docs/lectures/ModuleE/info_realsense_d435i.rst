.. _doc_info_realsense_d435i:


INFO - Realsense D435i
==================================



Install dependencies
--------------------

1. Make Ubuntu up-to-date including the latest stable kernel

   .. code-block:: bash

       sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade

2. Install the core packages required to build librealsense binaries and the affected kernel modules

   .. code-block:: bash

       sudo apt-get install libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libgtk-3-dev

   .. note::

      Certain librealsense CMake flags (e.g. CUDA) require CMake 3.8+ which may not be available via the default apt repositories on some Ubuntu LTS releases.

3. Install build tools

   .. code-block:: bash

       sudo apt-get install git wget cmake build-essential

4. Prepare Linux backend and development environment

   Unplug any connected RealSense camera and run

   .. code-block:: bash

       sudo apt-get install libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev

Install librealsense2
--------------------

Clone/Download the latest stable version of librealsense2 in one of the following ways:

- Clone the librealsense repo

  .. code-block:: bash

      git clone https://github.com/realsenseai/librealsense.git

- Download and unzip the latest stable librealsense2 version from the master branch:

  `RealSense.zip <https://github.com/realsenseai/librealsense/archive/master.zip>`_

Install the video for Linux (v4l2) driver

.. code-block:: bash

    sudo apt install v4l-utils

Run the RealSense permissions script from the librealsense root directory:

.. code-block:: bash

    cd librealsense
    ./scripts/setup_udev_rules.sh

.. note::

    You can remove the permissions by running:

    ./scripts/setup_udev_rules.sh --uninstall

Build and apply patched kernel modules for:

- Ubuntu 20/22/24 (focal/jammy/noble) with LTS kernel 5.15, 5.19, 6.5

  .. code-block:: bash

      ./scripts/patch-realsense-ubuntu-lts-hwe.sh

- Ubuntu 20 with LTS kernel (< 5.13)

  .. code-block:: bash

      ./scripts/patch-realsense-ubuntu-lts.sh

.. note::

   The script(s) above will download, patch and build realsense-affected kernel modules (drivers), then attempt to insert the patched module instead of the active one. If insertion fails the original uvc modules will be restored.

Check the patched modules installation by examining the generated log and inspecting the latest entries in the kernel log:

.. code-block:: bash

    sudo dmesg | tail -n 50

The log should indicate that a new _uvcvideo_ driver has been registered. Refer to `Troubleshooting <#troubleshooting-installation-and-patch-related-issues>`_ in case of errors/warning reports.

Other Resources
-------------------- 

Product page: `Intel RealSense D435i <https://www.realsenseai.com/products/depth-camera-d435i/>`_

Getting started: `RealSense developer docs <https://dev.realsenseai.com/docs/docs-get-started>`_