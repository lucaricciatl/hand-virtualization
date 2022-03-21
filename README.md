colcon build --symlink-install  // build pkg and create symbolic links

run files ->    python3 /home/atled/dev_ws/src/esp32_SerialInterface/esp32_SerialInterface/interface.py && python3 /home/atled/dev_ws/src/DataElaborationAndFiltering/      DataElaborationAndFiltering/filter_data.py && python3 /home/atled/dev_ws/src/hand_pose_publisher/hand_pose_publisher/publishpose.py && python3 /home/atled/dev_ws/src/kinematic_optimizer/kinematic_optimizer/kinematic_optimizer.py

run all --> python3 /home/atled/dev_ws/src/launch_system/launch_system/system.launch.py

LOG RAW DATA --> ros2 topic echo MPU{@}_rawdata                     //@ : imu identifier
LOG RAW DATA --> ros2 topic echo RightHandIndex{@}                  //@ : joint number
LOG RAW DATA --> ros2 topic echo RightHandIndexRot                  //first joint rot
LOG RAW DATA --> ros2 topic echo RightHand_pos                      //read position
LOG RAW DATA --> ros2 topic echo RightHand_rot                      //read angles
LOG RAW DATA --> ros2 topic echo MPU_{@}_filtered_pose_and_asset    //@ : read mpu SRD