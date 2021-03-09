// https://github.com/richardghirst/PiBits/blob/master/MPU6050-Pi-Demo/demo_dmp.cpp
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <signal.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/transform_datatypes.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;
MPU6050 accelgyro(0x68); // <-- use for AD0 high

int16_t ax, ay, az,gx, gy, gz;

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
bool ado = false;

//int sample_rate = 10;

void mySigintHandler(int sig){
	ROS_INFO("Shutting down mpu6050_node...");

	accelgyro.reset();

	// All the default sigint handler does is call shutdown()
	ros::shutdown();
}

///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
      printf("buff_ax = %ld  |  ", buff_ax);
      printf("buff_ay = %ld  |  ", buff_ay);
      printf("buff_az = %ld\n\n", buff_az);
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    //printf("...");
    printf("abs(mean_ax) = %d  |  ", abs(mean_ax));
    printf("abs(mean_ay) = %d  |  ", abs(mean_ay));
    printf("abs(mean_ax) az = %d\n", abs(16384-mean_ax));

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop(ros::NodeHandle pn, ros::NodeHandle n) {
  if (state==0){
    printf("\nReading sensors for first time...\n");
    meansensors();
    state++;
    delay(1000);
  }

  if (state==1) {
    printf("\nCalculating offsets...\n");
    calibration();
    state++;
    delay(1000);
  }

  if (state==2) {
    meansensors();
    printf("\nFINISHED!\n");
    printf("\nSensor readings with offsets:\n");
    printf("mean_ax = %d\n", mean_ax); 
    printf("mean_ay = %d\n", mean_ay); 
    printf("mean_az = %d\n", mean_az); 
    printf("mean_gx = %d\n", mean_gx); 
    printf("mean_gy = %d\n", mean_gy); 
    printf("mean_gz = %d\n", mean_gz);
    printf("\nYour offsets:\n");
    printf("ax_offset = %d\n", ax_offset); 
    printf("ay_offset = %d\n", ay_offset); 
    printf("az_offset = %d\n", az_offset); 
    printf("gx_offset = %d\n", gx_offset); 
    printf("gy_offset = %d\n", gy_offset); 
    printf("gz_offset = %d\n", gz_offset); 
    printf("\nNext data is printed as: acelX acelY acelZ giroX giroY giroZ\n");
    printf("Check that your sensor readings with offsets are close to 0 0 16384 0 0 0\n");
    printf("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)\n");
    while (1);
  }
}

int main(int argc, char **argv){

    ros::init(argc, argv, "mpu6050");

    // Allows parameters passed in via <param>
    ros::NodeHandle pn("~");

    // Does not allow parameters being passed in.
    ros::NodeHandle n;

    signal(SIGINT, mySigintHandler);

    ROS_INFO("Starting mpu6050_node...");

    pn.param<bool>("ado", ado, false);
    std::cout << "ADO: " << ado << std::endl << std::flush;

/*
    pn.param<int>("frequency", sample_rate, DEFAULT_SAMPLE_RATE_HZ);
	std::cout << "Using sample rate: " << sample_rate << std::endl;

    pn.param<std::string>("frame_id", frame_id, MPU_FRAMEID);
    std::cout << "Using frame_id: " << frame_id << std::endl;

    pn.param<int>("ax", ax, 0);
    pn.param<int>("ay", ay, 0);
    pn.param<int>("az", az, 0);
    pn.param<int>("gx", gx, 0);
    pn.param<int>("gy", gy, 0);
    pn.param<int>("gz", gz, 0);

    pn.param<bool>("ado", ado, false);
    std::cout << "ADO: " << ado << std::endl << std::flush;

    pn.param<bool>("debug", debug, false);
    std::cout << "Debug: " << debug << std::endl << std::flush;

    // NOISE PERFORMANCE: Power Spectral Density @10Hz, AFS_SEL=0 & ODR=1kHz 400 ug/√Hz (probably wrong)
    pn.param("linear_acceleration_stdev", linear_acceleration_stdev_, (400 / 1000000.0) * 9.807 );

    // Total RMS Noise: DLPFCFG=2 (100Hz) 0.05 º/s-rms (probably lower (?) @ 42Hz)
    pn.param("angular_velocity_stdev", angular_velocity_stdev_, 0.05 * (M_PI / 180.0));

    // 1 degree for pitch and roll
    pn.param("pitch_roll_stdev", pitch_roll_stdev_, 1.0 * (M_PI / 180.0));

    // 5 degrees for yaw
    pn.param("yaw_stdev", yaw_stdev_, 5.0 * (M_PI / 180.0));

    angular_velocity_covariance = angular_velocity_stdev_ * angular_velocity_stdev_;
    linear_acceleration_covariance = linear_acceleration_stdev_ * linear_acceleration_stdev_;
    pitch_roll_covariance = pitch_roll_stdev_ * pitch_roll_stdev_;
    yaw_covariance = yaw_stdev_ * yaw_stdev_;

*/

    // ================================================================
    // ===                      INITIAL SETUP                       ===
    // ================================================================

    // start message
    printf("\nMPU6050 Calibration Sketch\n");
    printf("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");

    printf("Initializing I2C...\n");
    I2Cdev::initialize();

    // verify connection
    printf("Testing device connections...\n");
    accelgyro = MPU6050(ado ? 0x69 : 0x68);
    if(accelgyro.testConnection()){
        std::cout << "MPU6050 connection successful" << std::endl << std::flush;
    }else{
        std::cout << "MPU6050 connection failed" << std::endl << std::flush;
        return 1;
    }

    usleep(100000);

    // initialize device
    printf("Initializing I2C devices...\n");
    accelgyro.initialize();

    usleep(100000);

    // reset offsets
    accelgyro.setXAccelOffset(713);
    accelgyro.setYAccelOffset(98);
    accelgyro.setZAccelOffset(17098);
    accelgyro.setXGyroOffset(0);
    accelgyro.setYGyroOffset(0);
    accelgyro.setZGyroOffset(0);

    usleep(100000);

/*
    // load and configure the DMP
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();
*/

/*
    // Set accel offsets.
    std::cout << "Setting X accel offset: " << ax << std::endl;
    mpu.setXAccelOffset(ax);
    std::cout << "Setting Y accel offset: " << ay << std::endl;
    mpu.setYAccelOffset(ay);
    std::cout << "Setting Z accel offset: " << az << std::endl;
    mpu.setZAccelOffset(az);

    // Set gyro offsets.
    std::cout << "Setting X gyro offset: " << gx << std::endl;
    mpu.setXGyroOffset(gx);
    std::cout << "Setting Y gyro offset: " << gy << std::endl;
    mpu.setYGyroOffset(gy);
    std::cout << "Setting Z gyro offset: " << gz << std::endl;
    mpu.setZGyroOffset(gz);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //printf(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
    }
    
    usleep(100000);

    imu_pub = n.advertise<sensor_msgs::Imu>("imu/data", 10);
    imu_euler_pub = n.advertise<geometry_msgs::Vector3Stamped>("imu/euler", 10);
	mag_pub = n.advertise<geometry_msgs::Vector3Stamped>("imu/mag", 10);
*/

    //ros::Rate r(sample_rate);
    while(ros::ok()){
        loop(pn, n);
        ros::spinOnce();
        //r.sleep();
    }

    std::cout << "Shutdown." << std::endl << std::flush;

    return 0;

}