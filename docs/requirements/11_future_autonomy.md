# 11. Future Autonomy

The long-term goal: object detection and avoidance. These requirements are **P3**. They're
written down now so that earlier work doesn't block them. None of this needs to be built until
the P1 and P2 requirements in domains 1–10 are verified.

## 11.1 Odometry

**11.1.1** The system shall measure wheel motion with an encoder on each drive wheel or motor, because R-Net does not report wheel speed. — P3 · I

**11.1.2** `wheelchair_odometry` shall publish `nav_msgs/Odometry` on `/odom` and the `odom → base_link` transform at 20 Hz or faster. — P3 · T

**11.1.3** Over a 10 m straight run on pavement, the odometry distance error should be under 2%. — P3 · T

## 11.2 Robot description

**11.2.1** A URDF shall describe the chair's geometry: wheel base, wheel radius, caster positions and sensor mounts. — P3 · I

**11.2.2** `robot_state_publisher` shall publish the chair's transforms from the URDF. — P3 · T

## 11.3 Object detection

**11.3.1** The system shall detect obstacles in front of the chair using a range sensor (lidar or depth camera). — P3 · D

**11.3.2** It shall detect an obstacle 0.3 m tall or taller at 2 m *(param)* in the direction of travel. — P3 · T

**11.3.3** Detected obstacles shall be published as a standard ROS type (e.g. `sensor_msgs/LaserScan` or a costmap). — P3 · I

## 11.4 Avoidance

**11.4.1** Safety shall stop the chair if an obstacle is within the stopping distance plus a 0.5 m margin *(param)* in the direction of travel. This applies in every mode, manual included. — P3 · T

**11.4.2** Avoidance shall reduce speed as obstacles get closer before it stops the chair. — P3 · T

**11.4.3** In Auto mode, the chair should steer around obstacles using Nav2. — P3 · D

**11.4.4** Detection and avoidance shall be added as inputs to Safety, without bypassing the command path in 1.3.2. — P3 · I

## 11.5 Implements (pseudo-tractor use)

**11.5.1** Implement actuators shall have their own command path. Implement commands shall not go through the drive `Twist` mux. — P3 · I

**11.5.2** A Safety stop shall also stop every implement actuator. — P3 · T

**11.5.3** Motor current while towing shall be logged, to set the over-current limit for drawbar loads (5.3.2). — P3 · T
