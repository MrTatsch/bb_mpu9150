phoenix_imu
===========

This repository creates a ROS package that publishes the Invensense MPU-9150 data into a ROS topic.

----

(The code uses the [linux-mpu9150](https://github.com/Pansenti/linux-mpu9150) driver.)
This packages was forked from [mpu9150_node](https://github.com/vmayoral/bb_mpu9150/blob/master/src/mpu9150_node.cpp)

---


####[mpu9150_node](https://github.com/vmayoral/bb_mpu9150/blob/master/src/mpu9150_node.cpp)
Get samples from the Invensense MPU9150 sensor and output these processed samples. Outputs can be either euler angles, quaternions, calibrated accelerometer or calibrated magnetometer.
Default values (these values should be changed at the local_defaults.h and afterwards the code should be cross-compiled again):
* Default I2C Bus: 1 (i2c-2 at the Beaglebone).
* Default sample rate: 10 Hz
* Default yaw mix factor: 4

#####Published topics
*imu_euler (std_msgs::String)*







