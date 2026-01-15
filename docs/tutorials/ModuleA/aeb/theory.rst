.. _doc_tutorials_aeb_theory:

AEB Theory of Operation
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


3️⃣ Time-to-Collision (TTC) vs. Distance-Based Braking
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When designing an autonomous braking system, you have two main approaches:

1. Braking based on **Distance to Object**
2. Braking based on **Time-to-Collision (TTC)**

Both methods aim to prevent collisions, but TTC is usually the superior choice. Let's break it down.

Distance-Based Braking (Threshold Approach)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This method triggers braking when an obstacle is closer than a set distance.

**How It Works**

- If an object is detected closer than X meters, apply brakes
- If the object is beyond X meters, continue driving

**Why This Can Be a Problem**

❌ **Speed Ignorance:**

- A slow-moving car at 2 m/s needs far less stopping distance than a car at 10 m/s
- A fixed threshold (e.g., "brake if object < 2m") doesn't scale with speed

❌ **Late Reactions at High Speed:**

- If a vehicle is moving fast, it may not have enough distance left to safely stop when the threshold is reached

❌ **Unnecessary Braking at Low Speed:**

- If a vehicle is moving very slowly, braking at the same fixed distance may be overly cautious, leading to unnecessary stops

Time-to-Collision (TTC) - A Smarter Alternative
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

TTC predicts how long until a collision happens if both objects maintain their current speeds.

**How It Works**

The Time-to-Collision (TTC) is calculated as:

.. image:: img/TTC_Calculation.jpg
   :alt: TTC Formula
   :width: 40%
   :align: center

|

- If TTC drops below a safe threshold (e.g., 0.5s), apply brakes
- If TTC is above a release threshold (e.g., 1.5s), allow normal driving

**Why TTC is Better**

✅ **Speed Awareness:**

- A vehicle at 2 m/s and a vehicle at 10 m/s will have different stopping distances, and TTC adapts braking accordingly

✅ **Smooth & Early Braking:**

- If an object is far but closing quickly, TTC detects the risk earlier than distance-based braking

✅ **No Unnecessary Stops:**

- If an object is close but not a threat (e.g., a parked car not moving into the path), TTC won't trigger braking unnecessarily

Example: Comparing Both Approaches
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Imagine a car moving at 10 m/s with an object 5 meters ahead.

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Method
     - Stopping Decision
   * - Distance-based braking (Threshold = 3m)
     - 🚗💥 Car doesn't brake until too late, leading to collision
   * - TTC-based braking (Threshold = 0.5s)
     - 🚗🛑 Car detects high closing speed and brakes early to avoid impact

Now, imagine the same scenario at 2 m/s:

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Method
     - Stopping Decision
   * - Distance-based braking (Threshold = 3m)
     - 🚗🛑 Unnecessary stop, because 3m is plenty of room at low speed
   * - TTC-based braking (Threshold = 0.5s)
     - 🚗✅ Car recognizes the slow approach and continues safely

Key Takeaways
^^^^^^^^^^^^^

- **Distance-based braking ignores speed** 🚗⚠️, which can cause late stops at high speeds or unnecessary stops at low speeds
- **TTC accounts for speed & closing rate** ⏳, making braking decisions more adaptive
- **TTC allows smoother driving** 🚀 because it avoids the jerky "brake-go-brake" behavior of fixed-distance thresholds

.. note::

   If you're designing an autonomous emergency braking system, TTC is the way to go! 🚀


4️⃣ Understanding Time to Collision (TTC)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**Time to Collision (TTC)** is a measure used in traffic safety to estimate the time it will take for two objects (usually vehicles) to collide if they continue on their current paths at their current speeds.

Basic TTC Concept
^^^^^^^^^^^^^^^^^

The fundamental TTC formula calculates how long until two objects collide based on their relative velocity:

.. image:: img/TTC_Calculation.jpg
   :alt: TTC Formula
   :width: 40%
   :align: center

|

- **Distance**: The current separation between the two objects
- **Relative Velocity (Δv)**: The rate at which the distance is changing

If the two objects are moving directly towards each other, Δv is simply the sum of their velocities. If they are moving in the same direction, Δv is the difference in their velocities.

Example Calculation
^^^^^^^^^^^^^^^^^^^

Consider two cars:

- **Car A** is stationary
- **Car B** is moving towards Car A at a speed of 20 m/s
- The **distance** between them is 100 meters

.. image:: img/ttc_example.jpg
   :alt: TTC Example Calculation
   :width: 50%
   :align: center

|

This means that if Car B continues at its current speed without slowing down or changing its path, it will collide with Car A in **5 seconds**.

.. note::

   The above calculation assumes constant speeds and straight-line paths. In real-world scenarios where speeds and directions can change, the calculation becomes more complex.


5️⃣ Time to Collision Using Laser Scan Data
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When using LiDAR data, we need to account for the angle of approach. Let's break it down:

Suppose you have two objects:

- **Object A**: Your vehicle (the RoboRacer)
- **Object B**: Another vehicle or obstacle

Given:

- **V**: The velocity of Object A
- **θ**: The angle between the direction of V and the line-of-sight between Object A and Object B
- **R**: Current distance between Object A and Object B (from LiDAR measurement)

The component of V that is directly along the line-of-sight is given by **V·cos(θ)**. This component is called the **"range-rate"**, and it represents how fast the distance between the two objects is changing.

.. image:: img/range-rate.png
   :alt: TTC Range Rate Formula
   :width: 40%
   :align: center

Calculating Range Rate
^^^^^^^^^^^^^^^^^^^^^^

The range rate **ṙ** represents how fast the distance is changing along each scan beam. A **positive** range rate means the distance is expanding (moving away), and a **negative** one means the distance is shrinking (approaching).

There are two methods to calculate range rate:

1. **Using Velocity Projection** (Recommended)

   Calculate by projecting the vehicle's longitudinal velocity onto each scan beam's angle:

   **ṙ = vₓ · cos(θᵢ)**

   Where:

   - **vₓ**: Vehicle's longitudinal velocity (from ``Odometry`` message)
   - **θᵢ**: Angle of the scan beam (determined from ``LaserScan`` angle parameters)

   Be careful with the sign: for a vehicle moving forward toward an obstacle directly ahead, the range rate should be negative since the distance is decreasing.

2. **Using Range Difference**

   Take the difference between consecutive range measurements:

   **ṙ = (r_current - r_previous) / Δt**

   Where **Δt** is the time elapsed between measurements (available from message timestamps).

Understanding the Negation and Operator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The negation in the iTTC formula (**-ṙ**) correctly interprets whether the range is decreasing or increasing:

- For a vehicle traveling **toward** an obstacle: range rate is **negative** (distance shrinking) → **-ṙ** becomes **positive**
- For a vehicle traveling **away** from an obstacle: range rate is **positive** (distance expanding) → **-ṙ** becomes **negative**

The operator **{x}₊ = max(x, 0)** ensures iTTC calculations are meaningful:

- When **-ṙ > 0** (approaching): iTTC is calculated normally
- When **-ṙ ≤ 0** (moving away or parallel): iTTC goes to infinity (no collision)

Implementing iTTC for AEB
^^^^^^^^^^^^^^^^^^^^^^^^^^

After calculating iTTC for all scan beams, you'll have an array of iTTC values corresponding to each angle. When a time to collision drops below a certain threshold, it means a collision is imminent and the vehicle should brake.

.. note::

   Don't forget to handle ``inf`` and ``nan`` values in your arrays, as these commonly appear when:

   - No obstacle is detected (range exceeds max range)
   - The vehicle is moving away from obstacles
   - Division by zero occurs


6️⃣ Automatic Emergency Braking with iTTC
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For this lab, you will make a Safety Node that should halt the car before it collides with obstacles. To do this, you will make a ROS 2 node that subscribes to the ``LaserScan`` and ``Odometry`` messages. It should analyze the ``LaserScan`` data and, if necessary, publish an ``AckermannDriveStamped`` with the ``speed`` field set to 0.0 m/s to brake.

After you've calculated the array of iTTCs, you should decide how to proceed with this information. You'll have to decide how to threshold, and how to best remove false positives (braking when collision isn't imminent). Don't forget to deal with ``inf``s or ``nan``s in your arrays.

**Topic Names**:

- ``LaserScan``: ``/scan``
- ``Odometry``: ``/odom`` (specifically, the longitudinal velocity can be found in ``twist.twist.linear.x``)
- ``AckermannDriveStamped``: ``/drive``
