.. _doc_tutorials_aeb_scan:



Understanding ``/scan`` Output (LaserScan)
==========================================

The ``/scan`` topic publishes lidar data using the ``sensor_msgs/msg/LaserScan`` message. The most important field for Automatic Emergency Braking (AEB) is ``ranges[]`` — an array of distance measurements (in meters) taken at evenly spaced angles across the lidar’s field of view.

What ``ranges[]`` Means
-----------------------

- ``ranges[]`` is an array of distance readings in **meters**.
- Each element in the array corresponds to **one laser ray** at a specific angle.
- The scan spans from ``angle_min`` to ``angle_max`` in steps of ``angle_increment``.

From the example output:

- ``angle_min = -2.356 rad``  (≈ **-135°**)
- ``angle_max =  2.356 rad``  (≈ **+135°**)
- ``angle_increment = 0.004363 rad`` (≈ **0.25° per sample**)

This is about a **270° field of view**.

How Many Samples Are In One Scan?
---------------------------------

The number of samples in ``ranges[]`` is approximately:

.. math::

   N \approx \frac{\text{angle\_max} - \text{angle\_min}}{\text{angle\_increment}} + 1

With the values above, this is approximately **1081 points** per scan.

Mapping an Index to an Angle
----------------------------

To find the angle of a specific distance measurement:

.. math::

   \text{angle}(i) = \text{angle\_min} + i \cdot \text{angle\_increment}

- ``ranges[0]`` is at ``angle_min`` (far left of the scan)
- ``ranges[last]`` is at ``angle_max`` (far right of the scan)

"Straight Ahead" (Angle 0) Index
--------------------------------

The index closest to **straight ahead** (angle = 0 radians) is:

.. math::

   i_{\text{forward}} \approx \frac{0 - \text{angle\_min}}{\text{angle\_increment}}

For this scan, the forward index is approximately **540** (near the middle of the array).

Interpreting Range Values
-------------------------

Example ``ranges[]`` values and what they imply:

- Values like ``0.75``, ``0.69``, ``0.68`` indicate obstacles **very close** (under 1 meter).
- Values like ``1.8`` to ``2.3`` indicate objects around **2 meters** away.
- Many lidars represent "no return" readings as ``inf`` or sometimes as a very large number.

In the example output, there is a value like ``65.53``, which is **greater than** ``range_max = 30.0``. For AEB logic, values above ``range_max`` should be treated as **invalid** and ignored.

Scan Timing
-----------

The timing fields indicate how quickly new scans arrive:

- ``scan_time = 0.025 s`` → about **40 scans per second** (40 Hz)

This matters for AEB because it limits how often the system can update distance checks or compute collision metrics.

Useful Terminal Commands
------------------------

Show scan metadata (without spamming the full ``ranges[]`` list):

.. code-block:: bash

   ros2 topic echo /scan --once | egrep "frame_id|angle_|range_|scan_time"

Show only the first part of the message (helpful for quickly spotting ``ranges:``):

.. code-block:: bash

   ros2 topic echo /scan --once | sed -n '1,25p'
