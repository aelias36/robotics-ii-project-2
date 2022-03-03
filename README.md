# robotics-ii-project-2

A simulation of a mobile robot in a room with furniture was created, where this robot has multiple sensors including LIDAR and UWB. The robot position was estimated using linear and non-linear single-timestep multilateration, using a localization Kalman filter, and with SLAM. Another Kalman filter mapped the UWB anchor positions given the robot position, and LIDAR mapping of the room was also performed. A room covering algorithm was developed which can correctly navigate around the room even without perfect robot state knowledge.
