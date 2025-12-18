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

Other Resources
-------------------- 

Product page: `Intel RealSense D435i <https://www.realsenseai.com/products/depth-camera-d435i/>`_

Getting started: `RealSense developer docs <https://dev.realsenseai.com/docs/docs-get-started>`_