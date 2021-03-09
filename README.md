# ros_mpu6050_calibration for Raspberry Pi
ROS package for mpu6050_calibration and test
Requires a Raspeberry PI 2, 3 or 4

This package was based in the following works:

The ROS package ros_mpu6050_node - https://github.com/chrisspen/ros_mpu6050_node  
MPU-6050 calibration sketch, by Luis Ródenas, originally posted here: https://www.i2cdevlib.com/forums/topic/96-arduino-sketch-to-automatically-calculate-mpu6050-offsets/  

Depends on the following software:
- I2Cdevlib
- bcm2835

# Install I2Cdevlib (chrisspen fork recomended):

sudo mkdir -p /usr/share/arduino/libraries  
cd /usr/share/arduino/libraries  
sudo git clone https://github.com/chrisspen/i2cdevlib.git  

# Install Bcm2835:

cd /tmp  
wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.50.tar.gz  
tar zxvf bcm2835-1.50.tar.gz  
cd bcm2835-1.50  
./configure  
make  
sudo make check  
sudo make install  

# download and make the ros package

cd ~/catkin_ws/src  
git clone https://github.com/inaciose/ros_mpu6050_calibration  
cd ..  
catkin_make  


# Must be run as root.

For mpu6050 calibration run:  
sudo bash -c "source /opt/ros/kinetic/setup.bash; source /home/ubuntu/catkin_ws/devel/setup.bash; roslaunch ros_mpu6050_calibration mpucal.launch"  

For mpu6050 test node run:  
sudo bash -c "source /opt/ros/kinetic/setup.bash; source /home/ubuntu/catkin_ws/devel/setup.bash; roslaunch ros_mpu6050_calibration mpu.launch"  

# For Arduino based mpu6050 calibration

https://github.com/melikabarzegaran/mpu6050-raw-calibration  
https://codebender.cc/sketch:97892#MPU6050%20calibration.ino  

