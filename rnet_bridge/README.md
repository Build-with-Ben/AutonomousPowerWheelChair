# rnet_bridge

Bridge between ROS 2 and the wheelchair's R-Net CAN bus. Converts the final safe velocity command (`/cmd_vel/safe`) into R-Net joystick frames sent to `/to_can_bus`, and decodes R-Net status frames (battery, motor current, heartbeat) into ROS topics.
