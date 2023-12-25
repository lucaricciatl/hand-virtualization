# vr hand 
the project utilizes several components for its implementation:

- A MPU-6050 with 6 degrees of freedom (3 dof accelerometer + 3 dof gyroscope).
- Two MPU-9250s with 9 degrees of freedom (3 dof accelerometer + 3 dof gyroscope + 3 dof magnetometer).
- A TCA9548A multiplexer to enable the use of multiple IMUs on the same I2C channel.
- A programmable ESP32 board with Arduino.

## IMU Characterization

An IMU is an inertial measurement unit with three accelerometers and three gyroscopes placed on the three axes of interest (body axes).

### Accelerometer

Accelerometer data allows the inference of roll and pitch of the IMU. However, to compensate for errors, sensor fusion with other sensors is necessary.

The formulas are:


\phi_a = \tan^{-1} \left( \frac{a_y}{a_z} \right)
\theta_a = \tan^{-1} \left( \frac{-a_x}{a_y \sin{\phi_a} + a_z \cos{\phi_a}} \right)

### Gyroscope

The gyroscope provides angular velocity on the three axes. Integration of these measures allows obtaining relative angles over time. To compensate for drifting, accelerometer measurements are used as correction.

The integration formulas are:


\dot{\phi} = p + q \sin{\phi} \tan{\theta} + r \cos{\phi} \tan{\theta}
\dot{\theta} = q \cos{\phi} - r \sin{\phi}
\dot{\psi} = q \sin{\phi} \sec{\theta} + r \cos{\phi} \sec{\theta}
## AHRS Characterization

An AHRS incorporates a magnetometer into the IMU, enabling the measurement of heading and yaw.

Magnetometer Data
### Magnetometer

The heading measurement from the magnetometer contributes to the calculation of yaw. The formula is:


\psi_m = \tan^{-1} \left( \frac{m_x \sin{\phi_a} - m_y \cos{\phi_a}}{m_x \cos{\theta_a} + m_y \sin{\theta_a} \sin{\phi_a} + m_z \sin{\theta_a} \cos{\phi_a}} \right)
IMU Placement

The MPU-9250s, equipped with a magnetometer, are positioned on the back of the hand (MPU0) and on the proximal phalanx (MPU1) to visualize yaw and rotations relative to the palm. The MPU-6050 is positioned on the distal phalanx (MPU2).

![alt text](https://github.com/ATLED-3301/vrhand/blob/main/Relazione%20SGN%20-%20VR%20hand/immagini/confronto/2_real.jpg)
![alt text](https://github.com/ATLED-3301/vrhand/blob/main/Relazione%20SGN%20-%20VR%20hand/immagini/confronto/2_sym.jpg)
![alt text](https://github.com/ATLED-3301/vrhand/blob/main/Relazione%20SGN%20-%20VR%20hand/immagini/confronto/7_real.jpg)
![alt text](https://github.com/ATLED-3301/vrhand/blob/main/Relazione%20SGN%20-%20VR%20hand/immagini/confronto/7_sym.jpg)

# setup
- colcon build --symlink-install  // build pkg and create symbolic links

- run files ->    python3 /home/atled/dev_ws/src/esp32_SerialInterface/esp32_SerialInterface/interface.py && python3 /home/atled/dev_ws/src/DataElaborationAndFiltering/      DataElaborationAndFiltering/filter_data.py && python3 /home/atled/dev_ws/src/hand_pose_publisher/hand_pose_publisher/publishpose.py && python3 /home/atled/dev_ws/src/kinematic_optimizer/kinematic_optimizer/kinematic_optimizer.py

- run all --> python3 /home/atled/dev_ws/src/launch_system/launch_system/system.launch.py

# Unity

we used unity to visualize the components of the hand.
After filtering the data through the set of attitude estimators, the estimates are sent to a set of relative orientation estimators. Their purpose is to calculate the joint angle of the hand's phalanges by taking the attitude of two successive links as input and calculating their relative rotation.
Joint angles are calculated, messages related to individual joint angles and the hand's overall attitude are sent via ROS2 to a graphical visualization software.

To validate the results, a simulation was set up in Unity3D, allowing real-time visualization of attitude estimation results through C# code integrated into Unity.

Once joint angle estimates were available in Unity, the values were transformed into quaternions (widely used in 3D graphics software due to lower computational cost). Between two samples, a programmable velocity spherical linear interpolation (SLERP) was performed, improving the animation's smoothness.

For result validation, motion tracking considerations were exclusively used. By moving the glove, we considered the estimation good when the virtual hand followed the movement.

Due to the conventions used in the filter and those used by Unity, the validity of the simulation lies in angles between 0 and 180 degrees. Beyond these angles, there are orientation changes in the virtual hand. Given the excessively imprecise estimate of the angle $\alpha_1$, it was kept at 0.


LOG RAW DATA --> ros2 topic echo MPU{@}_rawdata                     //@ : imu identifier
LOG RAW DATA --> ros2 topic echo RightHandIndex{@}                  //@ : joint number
LOG RAW DATA --> ros2 topic echo RightHandIndexRot                  //first joint rot
LOG RAW DATA --> ros2 topic echo RightHand_pos                      //read position
LOG RAW DATA --> ros2 topic echo RightHand_rot                      //read angles
LOG RAW DATA --> ros2 topic echo MPU_{@}_filtered_pose_and_asset    //@ : read mpu SRD
