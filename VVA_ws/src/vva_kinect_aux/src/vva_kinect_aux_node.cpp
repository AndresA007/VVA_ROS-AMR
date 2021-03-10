/* 
 * Copyright (c) 2013, Murilo F. M. <muhrix AT gmail DOT com>
 * Copyright (c) 2020, Andres A. <andres.arboleda AT gmail DOT com>
 * All rights reserved.
 *
 * Released under the BSD License.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *    This product includes software developed by the <COPYRIGHT HOLDER>.
 * 4. Neither the name of the <COPYRIGHT HOLDER> nor the
 *    names of its contributors may be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY <COPYRIGHT HOLDER> ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * Based on the code of "kinect_aux" published by:
 * Murilo F. M. <muhrix AT gmail DOT com>
 * 
 * Github repo of original code:
 * https://github.com/muhrix/kinect_aux/
 * 
 * This version modified by:
 * andres.arboleda AT gmail DOT com, (may/2020)
 */

#include <libusb-1.0/libusb.h>
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>

// VID and PID for Kinect and motor/acc/leds
#define MS_MAGIC_VENDOR 0x45e
#define MS_MAGIC_MOTOR_PRODUCT 0x02b0
// Constants for accelerometers
#define GRAVITY 9.80665
#define FREENECT_COUNTS_PER_G 819.
// The kinect can tilt from +31 to -31 degrees in what looks like 1 degree increments
// The control input looks like 2*desired_degrees
#define MAX_TILT_ANGLE 31.
#define MIN_TILT_ANGLE (-31.)

ros::Publisher pub_imu;
ros::Publisher pub_tilt_angle;
ros::Publisher pub_tilt_status;
ros::Publisher pub_tilt_joint_state;

ros::Subscriber sub_tilt_angle;
ros::Subscriber sub_led_option;
 
libusb_device_handle *dev(0);

double y_joint_status_array[10] = { 0 };
double x_joint_status_array[10] = { 0 };
int y_jsa_last_position = 0;
int x_jsa_last_position = 0;


/*******************
 * openAuxDevice
 *******************/
void openAuxDevice(int index = 0)
{
	libusb_device **devs; //pointer to pointer of device, used to retrieve a list of devices
	ssize_t cnt = libusb_get_device_list (0, &devs); //get the list of devices
	if (cnt < 0)
	{
		ROS_ERROR("No device on USB");
		return;
	}
	
	int nr_mot(0);
	for (int i = 0; i < cnt; ++i)
	{
		struct libusb_device_descriptor desc;
		const int r = libusb_get_device_descriptor (devs[i], &desc);
		if (r < 0)
			continue;

		// Search for the aux
		if (desc.idVendor == MS_MAGIC_VENDOR && desc.idProduct == MS_MAGIC_MOTOR_PRODUCT)
		{
			// If the index given by the user matches our camera index
			if (nr_mot == index)
			{
				if ((libusb_open (devs[i], &dev) != 0) || (dev == 0))
				{
					ROS_ERROR_STREAM("Cannot open aux " << index);
					return;
				}
				// Claim the aux
				libusb_claim_interface (dev, 0);
				break;
			}
			else
				nr_mot++;
		}
	}

	libusb_free_device_list (devs, 1);  // free the list, unref the devices in it
}


/*******************
 * publishState
 *******************/
void publishState(void)
{
	uint8_t buf[10];
	const int ret = libusb_control_transfer(dev, 0xC0, 0x32, 0x0, 0x0, buf, 10, 0);
	if (ret != 10)
	{
		ROS_ERROR_STREAM("Error in accelerometer reading, libusb_control_transfer returned " << ret);
		ros::shutdown();
	}
	
	const uint16_t ux = ((uint16_t)buf[2] << 8) | buf[3];
	const uint16_t uy = ((uint16_t)buf[6] << 8) | buf[7];
	const uint16_t uz = ((uint16_t)buf[4] << 8) | buf[5];
	
	const int16_t accelerometer_x = (int16_t)ux;
	const int16_t accelerometer_y = (int16_t)uy;
	const int16_t accelerometer_z = (int16_t)uz;
	const int8_t tilt_angle = (int8_t)buf[8];
	const uint8_t tilt_status = buf[9];
  
	// publish IMU
	sensor_msgs::Imu imu_msg;
	if (pub_imu.getNumSubscribers() > 0)
	{
		imu_msg.header.stamp = ros::Time::now();
		imu_msg.linear_acceleration.x = (double(accelerometer_x)/FREENECT_COUNTS_PER_G)*GRAVITY;
		imu_msg.linear_acceleration.y = (double(accelerometer_y)/FREENECT_COUNTS_PER_G)*GRAVITY;
		imu_msg.linear_acceleration.z = (double(accelerometer_z)/FREENECT_COUNTS_PER_G)*GRAVITY;
		imu_msg.linear_acceleration_covariance[0] = imu_msg.linear_acceleration_covariance[4]
			= imu_msg.linear_acceleration_covariance[8] = 0.01; // @todo - what should these be?
		imu_msg.angular_velocity_covariance[0] = -1; // indicates angular velocity not provided
		imu_msg.orientation_covariance[0] = -1; // indicates orientation not provided
		pub_imu.publish(imu_msg);
	}
	
	// publish tilt angle and status
	if (pub_tilt_angle.getNumSubscribers() > 0)
	{
		std_msgs::Float64 tilt_angle_msg;
		tilt_angle_msg.data = double(tilt_angle) / 2.;
		pub_tilt_angle.publish(tilt_angle_msg);
	}
	if (pub_tilt_status.getNumSubscribers() > 0)
	{
		std_msgs::UInt8 tilt_status_msg;
		tilt_status_msg.data = tilt_status;
		pub_tilt_status.publish(tilt_status_msg);
	}
  
  // publish joint state
  sensor_msgs::JointState joints_state;
	if (pub_tilt_joint_state.getNumSubscribers() > 0)
	{
    joints_state.header.stamp = ros::Time::now();
    joints_state.name.resize(3);
    joints_state.position.resize(3);
    
    double sum = 0;
    double joint_status_average = 0;

    joints_state.name[0] = "kinect_joint";
    // linear_acceleration ranges [0-10] and is converted to radians equivalency [0-pi/2]
    // Since the accelerometer values are very unstable, we will get the average of the last 5 values.
    y_joint_status_array[y_jsa_last_position] = - (double(accelerometer_y)/FREENECT_COUNTS_PER_G)*GRAVITY * (1.57/10.0);
    if (y_jsa_last_position == 9)
      y_jsa_last_position = 0;
    else
      y_jsa_last_position ++;
    // Find the average
    sum = 0;
    for(int i = 0; i < 10; i++) {
        sum += y_joint_status_array[i];
    }
    joint_status_average = sum / 10;
    // Round the value to only 1 decimal digit
    joints_state.position[0] = ((double)((int)(joint_status_average * 10 + .5))) / 10;
    
    joints_state.name[1] = "base_balancing_x_joint";
    x_joint_status_array[x_jsa_last_position] = (double(accelerometer_x)/FREENECT_COUNTS_PER_G)*GRAVITY * (1.57/10.0);
    if (x_jsa_last_position == 9)
      x_jsa_last_position = 0;
    else
      x_jsa_last_position ++;
    sum = 0;
    for(int i = 0; i < 10; i++) {
        sum += x_joint_status_array[i];
    }
    joint_status_average = sum / 10;
    joints_state.position[1] = ((double)((int)(joint_status_average * 10 + .5))) / 10;
    
    joints_state.name[2] = "base_link_y_axis_joint";
    // The set tilt angle is not relative to the base of the Kinect, it is absolute according to the measure of the accelerometer_y
    // There is no way to know the angle between the Kinect and the base unless we already know the inclination of the base.
    // This joint status should be published by an IMU fixed in the base.
    joints_state.position[2] = 0;
    
    pub_tilt_joint_state.publish(joints_state);
	}
}


/*******************
 * setTiltAngle
 *******************/
void setTiltAngle(const std_msgs::Float64 angleMsg)
{
	uint8_t empty[0x1];
	double angle(angleMsg.data);
  uint16_t angle_int;

	angle = (angle<MIN_TILT_ANGLE) ? MIN_TILT_ANGLE : ((angle>MAX_TILT_ANGLE) ? MAX_TILT_ANGLE : angle);
  
	angle = angle * 2;
  
  if (angle < 0)
		angle_int = (uint16_t)(65536 + angle);
  else
		angle_int = (uint16_t)angle;
  
	const int ret = libusb_control_transfer(dev, 0x40, 0x31, angle_int, 0x0, empty, 0x0, 0);
  
	if (ret != 0)
	{
		ROS_ERROR_STREAM("Error in setting tilt angle, libusb_control_transfer returned " << ret);
		ros::shutdown();
	}
}


/*******************
 * setLedOption
 *******************/
void setLedOption(const std_msgs::UInt16 optionMsg)
{
	uint8_t empty[0x1];
	const uint16_t option(optionMsg.data);
	
	const int ret = libusb_control_transfer(dev, 0x40, 0x06, (uint16_t)option, 0x0, empty, 0x0, 0);
	if (ret != 0)
	{
		ROS_ERROR_STREAM("Error in setting LED options, libusb_control_transfer returned " << ret);
		ros::shutdown();
	}
}


/*******************
 * main
 *******************/
int main(int argc, char* argv[])
{
  ROS_INFO("Start vva_kinect_aux");
	int ret = libusb_init(0);
	if (ret)
	{
		ROS_ERROR_STREAM("Cannot initialize libusb, error: " << ret);
		return 1;
	}
	
	ros::init(argc, argv, "vva_kinect_aux");
	ros::NodeHandle n;
	
  double my_rate;
  n.param<double>("rate", my_rate, 10);
  
	int deviceIndex;
	n.param<int>("device_index", deviceIndex, 0);
	openAuxDevice(deviceIndex);
	if (!dev)
	{
		ROS_ERROR_STREAM("No valid aux device found");
		libusb_exit(0);
		return 2;
	}
	
	pub_imu = n.advertise<sensor_msgs::Imu>("vva_kinect/imu", 15);
	pub_tilt_angle = n.advertise<std_msgs::Float64>("vva_kinect/cur_tilt_angle", 15);
	pub_tilt_status = n.advertise<std_msgs::UInt8>("vva_kinect/cur_tilt_status", 15);
  pub_tilt_joint_state = n.advertise<sensor_msgs::JointState>("vva_kinect/tilt_joint_state", 15);
	
	sub_tilt_angle = n.subscribe("vva_kinect/tilt_angle", 1, setTiltAngle);
	sub_led_option = n.subscribe("vva_kinect/led_option", 1, setLedOption);
  
  // Set Kinect's initial position to tilt=0
  std_msgs::Float64 initial_tilt;
  initial_tilt.data = double(0);
  setTiltAngle(initial_tilt);
  
  ros::Rate rate(my_rate);
	 
	while (ros::ok())
	{
		ros::spinOnce();
		publishState();
    rate.sleep();
	}
	
	libusb_exit(0);
  ROS_INFO("Stop vva_kinect_aux. CHAO PESCAO!");
	return 0;
}
