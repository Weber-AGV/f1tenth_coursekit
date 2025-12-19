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


5. Jetson Orin kernel builder: `jetson-orin-kernel-builder <https://github.com/jetsonhacks/jetson-orin-kernel-builder>`_

   Tools to build the Linux kernel and modules on board the Jetson AGX Orin, Orin Nano, or Orin NX. This tool is designed for beginning to intermediate users — please read the entire document in the repository before proceeding.

   This is for JetPack 6. Supporting video on YouTube:

   .. raw:: html

       <div class="video-container">
           <iframe width="560" height="315" src="https://www.youtube.com/embed/7P6I2jeJNYo" title="Jetson Orin Kernel Builder — YouTube" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
       </div>

   .. code-block:: bash

       git clone https://github.com/jetsonhacks/jetson-orin-kernel-builder.git
       cd jetson-orin-kernel-builder

   Getting kernel and module sources
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   Run the following from the repository root to fetch the kernel and module sources (adjust per README and your device):

   .. code-block:: bash

       ./scripts/get_kernel_sources.sh

   Edit the kernel-builder configuration using the provided GUI script (adjust options for your device):

   .. code-block:: bash

       ./scripts/edit_config_gui.sh

   You can use the GUI to set device and kernel options — a sample screenshot is shown below:

   .. image:: media/kernel_config_gui.png
      :alt: Kernel configuration GUI
      :align: center
      :width: 600px

    Ctrl+F to search for CH341

    .. image:: media/search_ch341.png
       :alt: Search for CH341
       :align: center
       :width: 480px

    Next, click the letter 'N' to change it to 'M' for the CH341 Single Port Driver (build as a module):

    .. image:: media/ch341_to_module.png
       :alt: Set CH341 to Module (M)
       :align: center
       :width: 480px

   Follow the repository README for any device-specific options and the recommended order for subsequent scripts.


6. Jetson Orin helper: JetsonHacks `jetson-orin-librealsense <https://github.com/jetsonhacks/jetson-orin-librealsense>`_

   Clone the JetsonHacks helper repository and follow its README for Jetson-specific build and install instructions:

   .. code-block:: bash

       git clone https://github.com/jetsonhacks/jetson-orin-librealsense.git
       cd jetson-orin-librealsense/build

   




.. Install librealsense2
.. --------------------

.. Clone/Download the latest stable version of librealsense2 in one of the following ways:

.. - Clone the librealsense repo

..   .. code-block:: bash

..       git clone https://github.com/realsenseai/librealsense.git

.. - Download and unzip the latest stable librealsense2 version from the master branch:

..   `RealSense.zip <https://github.com/realsenseai/librealsense/archive/master.zip>`_

.. Install the video for Linux (v4l2) driver

.. .. code-block:: bash

..     sudo apt install v4l-utils

.. Run the RealSense permissions script from the librealsense root directory:

.. .. code-block:: bash

..     cd librealsense
..     ./scripts/setup_udev_rules.sh

.. .. note::

..     You can remove the permissions by running:

..     ./scripts/setup_udev_rules.sh --uninstall

.. Build librealsense using the RSUSB backend (Jetson-friendly)

.. On Jetson systems, the RealSense kernel patch script targets Ubuntu generic kernels and will fail on the NVIDIA tegra kernel. Instead of modifying the kernel, librealsense can be built using its RSUSB backend, which communicates with the camera directly over USB from user space.

.. From the librealsense root directory:

.. .. code-block:: bash

..     cd ~/librealsense
..     mkdir -p build
..     cd build

..     cmake .. \
..      -DFORCE_RSUSB_BACKEND=true \
..      -DBUILD_EXAMPLES=true

..     make -j$(nproc)
..     sudo make install
..     sudo ldconfig

.. Check the patched modules installation by examining the generated log and inspecting the latest entries in the kernel log:

.. .. code-block:: bash

..     sudo dmesg | tail -n 50

.. The log should indicate that a new _uvcvideo_ driver has been registered. Refer to `Troubleshooting <#troubleshooting-installation-and-patch-related-issues>`_ in case of errors/warning reports.

Other Resources
-------------------- 

Installing Intel RealSense SDK 2.0: `Jetson Hacks install Intel RealSense <https://jetsonhacks.com/2025/03/20/jetson-orin-realsense-in-5-minutes/>`_

Product page: `Intel RealSense D435i <https://www.realsenseai.com/products/depth-camera-d435i/>`_

Getting started: `RealSense developer docs <https://dev.realsenseai.com/docs/docs-get-started>`_

.. raw:: html

    <div class="video-container">
        <iframe width="560" height="315" src="https://www.youtube.com/embed/FpMCJsg_KmE?si=qhhzguxsaBv_YG6D" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
    </div> 