/* 
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
 * Created by:
 * andres.arboleda AT gmail DOT com, (oct/2020)
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <Encoder.h>
#include <vva_msgs/MotorCalibrationParams.h>
#include "vva_motor_utils.h"
#include "vva_arduino_status_codes.h"


ros::NodeHandle _nh;

// Variables used for encoders angular vel publishing (background task)
///////////////////////////////////////////////////////////////

std_msgs::Float32 _lfwheel_angular_vel_enc_msg;
std_msgs::Float32 _lbwheel_angular_vel_enc_msg;
std_msgs::Float32 _rfwheel_angular_vel_enc_msg;
std_msgs::Float32 _rbwheel_angular_vel_enc_msg;
std_msgs::UInt8 _vva_arduino_status_msg;
vva_msgs::MotorCalibrationParams _motor_calibration_params_msg;

ros::Publisher _lfwheel_angular_vel_enc_pub("vva_angular_vel_left_front_wheel_enc", &_lfwheel_angular_vel_enc_msg);
ros::Publisher _lbwheel_angular_vel_enc_pub("vva_angular_vel_left_back_wheel_enc", &_lbwheel_angular_vel_enc_msg);
ros::Publisher _rfwheel_angular_vel_enc_pub("vva_angular_vel_right_front_wheel_enc", &_rfwheel_angular_vel_enc_msg);
ros::Publisher _rbwheel_angular_vel_enc_pub("vva_angular_vel_right_back_wheel_enc", &_rbwheel_angular_vel_enc_msg);
ros::Publisher _motor_calibration_params_pub("vva_motor_calibration_params_ul", &_motor_calibration_params_msg);
ros::Publisher _vva_arduino_status_pub("vva_arduino_status", &_vva_arduino_status_msg);

Encoder _left_front_encoder(20, 48);
Encoder _left_back_encoder(21, 53);
Encoder _right_front_encoder(18, 44);
Encoder _right_back_encoder(19, 46);

unsigned long _time_prev_update;

// Variables used for motor calibration
///////////////////////////////////////////////////////////////

// Flow control flags (states)
bool _calibrate_wheels_pending      = true;
bool _find_max_vel_pending          = false;
bool _wait_stopped_pending          = false;
bool _find_min_vel_pending          = false;

// Timer control flag
bool _reset_start_time_pending = true;

// Other global variables used on the motor calibration process
float _prev_angular_vel_right_front_wheel = 0.0;
unsigned long _start_time;
int _motor_cmd = 0;

// Increase step size used to find the min. velocity of the motor
const int _min_cmd_calibration_resolution = 10;

// Variables used during operation state
///////////////////////////////////////////////////////////////

float _lfwheel_angular_vel_control = 0;
float _lbwheel_angular_vel_control = 0;
float _rfwheel_angular_vel_control = 0;
float _rbwheel_angular_vel_control = 0;


/********************************************************
 * Subscribed topics and callback functions
 */
void motor_cal_params_cb( const vva_msgs::MotorCalibrationParams& motor_cal_params_msg) {
  if (motor_cal_params_msg.motor_cmd_max == 0) {
    // If 'motor_cmd_max' is 0, set the state flags to start the motors calibration
    _calibrate_wheels_pending = true;
    _find_max_vel_pending     = true;
    _wait_stopped_pending     = false;
    _find_min_vel_pending     = false;
    
    _motor_cmd = 0;
  }
  else {
    // Set the state flags to avoid the calibration
    _calibrate_wheels_pending = false;
    _find_max_vel_pending     = false;
    _wait_stopped_pending     = false;
    _find_min_vel_pending     = false;
    
    // Save the calibration parameters sent by the computer (cal. params are declared in vva_motor_utils.h)
    _motor_cmd_max = motor_cal_params_msg.motor_cmd_max;
    _motor_cmd_min = motor_cal_params_msg.motor_cmd_min;
    _motor_max_angular_vel = motor_cal_params_msg.motor_max_angular_vel;
    _motor_min_angular_vel = motor_cal_params_msg.motor_min_angular_vel;
  }
}

void lfwheel_angular_vel_control_cb( const std_msgs::Float32& msg) {
  _lfwheel_angular_vel_control = msg.data;
}

void lbwheel_angular_vel_control_cb( const std_msgs::Float32& msg) {
  _lbwheel_angular_vel_control = msg.data;
}

void rfwheel_angular_vel_control_cb( const std_msgs::Float32& msg) {
  _rfwheel_angular_vel_control = msg.data;
}

void rbwheel_angular_vel_control_cb( const std_msgs::Float32& msg) {
  _rbwheel_angular_vel_control = msg.data;
}

ros::Subscriber<vva_msgs::MotorCalibrationParams> _motor_calibration_params_sub("vva_motor_calibration_params_dl", &motor_cal_params_cb );
ros::Subscriber<std_msgs::Float32> _lfwheel_angular_vel_control_sub("vva_angular_vel_left_front_wheel_control", &lfwheel_angular_vel_control_cb );
ros::Subscriber<std_msgs::Float32> _lbwheel_angular_vel_control_sub("vva_angular_vel_left_back_wheel_control", &lbwheel_angular_vel_control_cb );
ros::Subscriber<std_msgs::Float32> _rfwheel_angular_vel_control_sub("vva_angular_vel_right_front_wheel_control", &rfwheel_angular_vel_control_cb );
ros::Subscriber<std_msgs::Float32> _rbwheel_angular_vel_control_sub("vva_angular_vel_right_back_wheel_control", &rbwheel_angular_vel_control_cb );



/********************************************************
 * setup
 */
void setup()
{
  init_motor_pins();
  
  _nh.initNode();
  
  _nh.advertise(_lfwheel_angular_vel_enc_pub);
  _nh.advertise(_lbwheel_angular_vel_enc_pub);
  _nh.advertise(_rfwheel_angular_vel_enc_pub);
  _nh.advertise(_rbwheel_angular_vel_enc_pub);
  _nh.advertise(_motor_calibration_params_pub);
  _nh.advertise(_vva_arduino_status_pub);

  _nh.subscribe(_motor_calibration_params_sub);
  _nh.subscribe(_lfwheel_angular_vel_control_sub);
  _nh.subscribe(_lbwheel_angular_vel_control_sub);
  _nh.subscribe(_rfwheel_angular_vel_control_sub);
  _nh.subscribe(_rbwheel_angular_vel_control_sub);
  
  _time_prev_update = millis();
}

/********************************************************
 * loop
 */
void loop()
{
  // Calculate and publish encoders angular velocity
  ///////////////////////////////////////////////////////////////
  
  unsigned long time_curr_update = millis();
  // Calculate the Delta of time in seconds
  float dt = (time_curr_update - _time_prev_update) / 1000.0;
  _time_prev_update = time_curr_update;
  
  float lfwheel_angular_vel_enc = enc_2_rads(_left_front_encoder.read()) / dt;
  float lbwheel_angular_vel_enc = enc_2_rads(_left_back_encoder.read()) / dt;
  float rfwheel_angular_vel_enc = enc_2_rads(_right_front_encoder.read()) / dt;
  float rbwheel_angular_vel_enc = enc_2_rads(_right_back_encoder.read()) / dt;
  
  _lfwheel_angular_vel_enc_msg.data = lfwheel_angular_vel_enc;
  _lbwheel_angular_vel_enc_msg.data = lbwheel_angular_vel_enc;
  _rfwheel_angular_vel_enc_msg.data = rfwheel_angular_vel_enc;
  _rbwheel_angular_vel_enc_msg.data = rbwheel_angular_vel_enc;
  
  _lfwheel_angular_vel_enc_pub.publish( &_lfwheel_angular_vel_enc_msg );
  _lbwheel_angular_vel_enc_pub.publish( &_lbwheel_angular_vel_enc_msg );
  _rfwheel_angular_vel_enc_pub.publish( &_rfwheel_angular_vel_enc_msg );
  _rbwheel_angular_vel_enc_pub.publish( &_rbwheel_angular_vel_enc_msg );

  _left_front_encoder.write(0);
  _left_back_encoder.write(0);
  _right_front_encoder.write(0);
  _right_back_encoder.write(0);

  // If required, calibrate the wheels motors limits
  ///////////////////////////////////////////////////////////////

  if (_calibrate_wheels_pending) {
    if (_find_max_vel_pending) {
      _vva_arduino_status_msg.data = CALIBRATION_FINDING_MAX_VEL;
      _vva_arduino_status_pub.publish(&_vva_arduino_status_msg);
      
      if (_reset_start_time_pending) {
        _start_time = millis();
        _reset_start_time_pending = false;
      }
      if ((millis() - _start_time) < 2000) {
        move_motor(LEFT_FRONT_WHEEL,  _motor_cmd_max);
        move_motor(LEFT_BACK_WHEEL,   _motor_cmd_max);
        move_motor(RIGHT_FRONT_WHEEL, _motor_cmd_max);
        move_motor(RIGHT_BACK_WHEEL,  _motor_cmd_max);

        if (rfwheel_angular_vel_enc > _prev_angular_vel_right_front_wheel) {
          _prev_angular_vel_right_front_wheel = rfwheel_angular_vel_enc;
        }
        else {
          _motor_max_angular_vel = _prev_angular_vel_right_front_wheel;
        }
      }
      else {

        _vva_arduino_status_msg.data = CALIBRATION_MAX_VEL_FOUND;
        _vva_arduino_status_pub.publish(&_vva_arduino_status_msg);
        
        _find_max_vel_pending = false;
        _wait_stopped_pending = true;
        _reset_start_time_pending = true;
      }
    }
    else if (_wait_stopped_pending) {
      _vva_arduino_status_msg.data = CALIBRATION_WAITING_TO_STOP_WHEELS;
      _vva_arduino_status_pub.publish(&_vva_arduino_status_msg);

      if (_reset_start_time_pending) {
        _start_time = millis();
        _reset_start_time_pending = false;
      }
      if ((millis() - _start_time) < 2000) {
        // Make sure the motors are stoped
        move_motor(LEFT_FRONT_WHEEL,  0);
        move_motor(LEFT_BACK_WHEEL,   0);
        move_motor(RIGHT_FRONT_WHEEL, 0);
        move_motor(RIGHT_BACK_WHEEL,  0);
      }
      else {
        _wait_stopped_pending = false;
        _find_min_vel_pending = true;
        _reset_start_time_pending = true;
      }
    }
    else if (_find_min_vel_pending) {
      _vva_arduino_status_msg.data = CALIBRATION_FINDING_MIN_VEL;
      _vva_arduino_status_pub.publish(&_vva_arduino_status_msg);
      
      if ((rfwheel_angular_vel_enc <= 0.1) && (_motor_cmd < _motor_cmd_max - _min_cmd_calibration_resolution)) {
        if (_reset_start_time_pending) {
          _start_time = millis();
          _reset_start_time_pending = false;
        }
        if ((millis() - _start_time) < 2000) {
          move_motor(LEFT_FRONT_WHEEL,  _motor_cmd);
          move_motor(LEFT_BACK_WHEEL,   _motor_cmd);
          move_motor(RIGHT_FRONT_WHEEL, _motor_cmd);
          move_motor(RIGHT_BACK_WHEEL,  _motor_cmd);

        }
        else {
          _motor_cmd += _min_cmd_calibration_resolution;
          _reset_start_time_pending = true;
        }
      }
      else {
        _vva_arduino_status_msg.data = CALIBRATION_MIN_VEL_FOUND;
        _vva_arduino_status_pub.publish(&_vva_arduino_status_msg);
        
        _motor_min_angular_vel = rfwheel_angular_vel_enc;
        _motor_cmd_min = _motor_cmd;
        _find_min_vel_pending = false;
        _calibrate_wheels_pending = false;
        
        move_motor(LEFT_FRONT_WHEEL,  0);
        move_motor(LEFT_BACK_WHEEL,   0);
        move_motor(RIGHT_FRONT_WHEEL, 0);
        move_motor(RIGHT_BACK_WHEEL,  0);

        // Publish the motor calibration parameters
        _motor_calibration_params_msg.motor_cmd_max = _motor_cmd_max;
        _motor_calibration_params_msg.motor_cmd_min = _motor_cmd_min;
        _motor_calibration_params_msg.motor_max_angular_vel = _motor_max_angular_vel;
        _motor_calibration_params_msg.motor_min_angular_vel = _motor_min_angular_vel;
        _motor_calibration_params_pub.publish( &_motor_calibration_params_msg );
      }
    }
    else {
      _vva_arduino_status_msg.data = CALIBRATION_PENDING_TO_CALIBRATE;
      _vva_arduino_status_pub.publish(&_vva_arduino_status_msg);
    }
  }

  // Once calibrated, start the operation. Receive angular vels and translate them to move the motors
  ///////////////////////////////////////////////////////////////
  
  else {
    if (_lfwheel_angular_vel_control != 0 || _lbwheel_angular_vel_control != 0 ||
           _rfwheel_angular_vel_control != 0 || _rbwheel_angular_vel_control != 0) {
      _vva_arduino_status_msg.data = RECEIVED_WHEEL_CONTROL_MESSAGE;
      _vva_arduino_status_pub.publish(&_vva_arduino_status_msg);
    }
    else {
      _vva_arduino_status_msg.data = OPERATION_STATE;
      _vva_arduino_status_pub.publish(&_vva_arduino_status_msg);
    }

    int lf_motor_cmd = angularvel_2_motorcmd( _lfwheel_angular_vel_control );
    int lb_motor_cmd = angularvel_2_motorcmd( _lbwheel_angular_vel_control );
    int rf_motor_cmd = angularvel_2_motorcmd( _rfwheel_angular_vel_control );
    int rb_motor_cmd = angularvel_2_motorcmd( _rbwheel_angular_vel_control );

    move_motor(LEFT_FRONT_WHEEL,  lf_motor_cmd);
    move_motor(LEFT_BACK_WHEEL,   lb_motor_cmd);
    move_motor(RIGHT_FRONT_WHEEL, rf_motor_cmd);
    move_motor(RIGHT_BACK_WHEEL,  rb_motor_cmd);
  }
  
  _nh.spinOnce();
  delay(95);
}
