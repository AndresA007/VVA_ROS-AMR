#!/usr/bin/env python

#------------------------------------------------------------------------------
# Copyright (c) 2020, Andres A. <andres.arboleda AT gmail DOT com>
# All rights reserved.
# 
# Released under the BSD License.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. All advertising materials mentioning features or use of this software
#    must display the following acknowledgement:
#    This product includes software developed by the <COPYRIGHT HOLDER>.
# 4. Neither the name of the <COPYRIGHT HOLDER> nor the
#    names of its contributors may be used to endorse or promote products
#    derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY <COPYRIGHT HOLDER> ''AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
# Created by:
# andres.arboleda AT gmail DOT com, (may/2020)
# Modified by:
# andres.arboleda AT gmail DOT com, (sep/2020)
# Modified by:
# andres.arboleda AT gmail DOT com, (oct/2020)
#------------------------------------------------------------------------------

import rospy
import os
import pickle

# Messages
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import UInt8
from vva_msgs.msg import MotorCalibrationParams


class CmdVelToMotorsCommand:
  def __init__(self):
    rospy.init_node('vva_base_controller')
    self.cmdvel_sub = rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
    
    self.vva_angular_vel_left_front_wheel_enc_sub = rospy.Subscriber('vva_angular_vel_left_front_wheel_enc',
                                             Float32, self.angular_vel_left_front_wheel_enc_callback)
    self.vva_angular_vel_left_back_wheel_enc_sub = rospy.Subscriber('vva_angular_vel_left_back_wheel_enc',
                                             Float32, self.angular_vel_left_back_wheel_enc_callback)
    self.vva_angular_vel_right_front_wheel_enc_sub = rospy.Subscriber('vva_angular_vel_right_front_wheel_enc',
                                             Float32, self.angular_vel_right_front_wheel_enc_callback)
    self.vva_angular_vel_right_back_wheel_enc_sub = rospy.Subscriber('vva_angular_vel_right_back_wheel_enc',
                                             Float32, self.angular_vel_right_back_wheel_enc_callback)
                                             
    self.vva_motor_calibration_params_sub = rospy.Subscriber('vva_motor_calibration_params_ul',
                                             MotorCalibrationParams, self.motor_calibration_params_callback)
    self.vva_arduino_status_sub = rospy.Subscriber('vva_arduino_status', UInt8, self.vva_arduino_status_callback)
    
    self.vva_angular_vel_left_front_wheel_control_pub = rospy.Publisher('vva_angular_vel_left_front_wheel_control', Float32, queue_size=10)
    self.vva_angular_vel_left_back_wheel_control_pub = rospy.Publisher('vva_angular_vel_left_back_wheel_control', Float32, queue_size=10)
    self.vva_angular_vel_right_front_wheel_control_pub = rospy.Publisher('vva_angular_vel_right_front_wheel_control', Float32, queue_size=10)
    self.vva_angular_vel_right_back_wheel_control_pub = rospy.Publisher('vva_angular_vel_right_back_wheel_control', Float32, queue_size=10)
    
    self.vva_motor_calibration_params_pub = rospy.Publisher('vva_motor_calibration_params_dl', MotorCalibrationParams, queue_size=10)
    

    self.L = rospy.get_param('~robot_wheel_separation_distance', 0.38)
    self.wheel_D = rospy.get_param('~robot_wheel_diameter', 0.13)
    self.calibration_file_path = rospy.get_param('~calibration_file_path', "wheels_calibration/wheels_calibration.txt")
    self.pid_on = rospy.get_param('~pid_on',True)
    self.rotation_Kp = rospy.get_param('~rotation_Kp', 1.0)
    self.rotation_Ki = rospy.get_param('~rotation_Ki', 1.0)
    self.rotation_Kd = rospy.get_param('~rotation_Kd', 1.0)
    self.translation_Kp = rospy.get_param('~translation_Kp', 1.0)
    self.translation_Ki = rospy.get_param('~translation_Ki', 1.0)
    self.translation_Kd = rospy.get_param('~translation_Kd', 1.0)
    self.rate = rospy.get_param('~rate', 10)
    self.timeout_idle = rospy.get_param('~timeout_idle', 50)
    
    self.time_prev_update = rospy.Time.now()

    # Target linear and angular speeds of the base
    self.target_v = 0
    self.target_w = 0
    
    # Angular speed of the wheels from encoders readings
    self.angular_vel_left_front_wheel_enc = 0
    self.angular_vel_left_back_wheel_enc = 0
    self.angular_vel_right_front_wheel_enc = 0
    self.angular_vel_right_back_wheel_enc = 0
    
    # PID control variables
    self.lfwheel_pid = {}
    self.lbwheel_pid = {}
    self.rfwheel_pid = {}
    self.rbwheel_pid = {}
    
    # Maximum and minimum motor command and wheel speeds.
    self.wheels_calibration_params = {
      'motor_cmd_max': 0,
      'motor_cmd_min': 0,
      'motor_max_angular_vel': 0.0,
      'motor_min_angular_vel': 0.0
    }
    
    # This variable has de code indicating the current status of the Arduino
    self.arduino_status = 0
    
    # Translation of arduino status to text
    self.arduino_status_translate = {
      10: 'Checking if calibration is required...',
      11: 'Calibrating, finding max. angular velocity...',
      12: 'Calibrating, max. velocity found.',
      13: 'Calibrating, waiting to stop wheels...',
      14: 'Calibrating, finding min. velocity...',
      15: 'Calibrating, min. velocity found.',
      21: 'Ready to operate.',
      22: 'Move wheels request received'
    }
    

    ## DEBUG
    # Debugging statistics file
    # ~ self.left_front_wheel_stats_file = None
    # ~ self.left_back_wheel_stats_file = None
    # ~ self.right_front_wheel_stats_file = None
    # ~ self.right_back_wheel_stats_file = None
    
    

  # ==================================================
  # Main loop
  # ==================================================
  # When given no commands for some time, do not move
  def spin(self):
    rospy.loginfo("vva_base_controller: Start")
    rate = rospy.Rate(self.rate)
    time_curr_update = rospy.Time.now()
    
    rospy.on_shutdown(self.shutdown)

    # Wait for the communication with the Arduino to be ready
    rospy.loginfo('vva_base_controller: Waiting for rosserial to establish communication with the Arduino...')
    while self.arduino_status == 0:
      rospy.sleep(0.2)
    rospy.loginfo('vva_base_controller: rosserial communication established.')
    
    # Load the wheels calibration parameters
    if os.path.exists(self.calibration_file_path):
      # Read the calibration parameters
      rospy.loginfo("vva_base_controller: Wheels calibration file found. Loading calibration parameters...")
      with open(self.calibration_file_path, 'r') as fc:
        self.wheels_calibration_params = pickle.load(fc)
      rospy.loginfo("vva_base_controller: Calibration parameters:")
      rospy.loginfo("vva_base_controller: motor_cmd_max: %d", self.wheels_calibration_params['motor_cmd_max'])
      rospy.loginfo("vva_base_controller: motor_cmd_min: %d", self.wheels_calibration_params['motor_cmd_min'])
      rospy.loginfo("vva_base_controller: motor_max_angular_vel: %.2f", self.wheels_calibration_params['motor_max_angular_vel'])
      rospy.loginfo("vva_base_controller: motor_min_angular_vel: %.2f", self.wheels_calibration_params['motor_min_angular_vel'])
      # Publish the calibration parameters to the Arduino
      motor_calibration_params_msg = MotorCalibrationParams()
      motor_calibration_params_msg.motor_cmd_max = self.wheels_calibration_params['motor_cmd_max']
      motor_calibration_params_msg.motor_cmd_min = self.wheels_calibration_params['motor_cmd_min']
      motor_calibration_params_msg.motor_max_angular_vel = self.wheels_calibration_params['motor_max_angular_vel']
      motor_calibration_params_msg.motor_min_angular_vel = self.wheels_calibration_params['motor_min_angular_vel']
      self.vva_motor_calibration_params_pub.publish(motor_calibration_params_msg)
    else:
      rospy.loginfo('vva_base_controller: Wheels calibration file not found. Starting calibration...')
      # Publishing calibration parameters with '0' values, makes the Arduino to start the calibration routine.
      motor_calibration_params_msg = MotorCalibrationParams()
      motor_calibration_params_msg.motor_cmd_max = 0
      motor_calibration_params_msg.motor_cmd_min = 0
      motor_calibration_params_msg.motor_max_angular_vel = 0.0
      motor_calibration_params_msg.motor_min_angular_vel = 0.0
      self.vva_motor_calibration_params_pub.publish(motor_calibration_params_msg)

    # Wait for the Arduino to confirm that the calibration parameters were correctly set
    rospy.loginfo('vva_base_controller: Checking if the Arduino has the calibration parameters set...')
    arduino_prev_status = 0
    while True:
      if self.arduino_status != arduino_prev_status:
        rospy.loginfo('Arduino: %s', self.arduino_status_translate[self.arduino_status])
        arduino_prev_status = self.arduino_status
      if self.arduino_status == 21:
        break;
      rospy.sleep(0.2)
    rospy.loginfo('vva_base_controller: Calibration parameters correctly set in the Arduino.')
    
    
    ## DEBUG
    # Save statistics about wheels target, control and encoder speeds
    # ~ self.left_front_wheel_stats_file =  open("/home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_base_controller/wheels_calibration/left_front_wheel_stats.csv", 'w')
    # ~ self.left_front_wheel_stats_file.write(
                  # ~ "vel_target_lineal," +
                  # ~ "vel_target_angular," +
                  # ~ "angular_vel_target_left_front_wheel," +
                  # ~ "angular_vel_encoder_left_front_wheel," +
                  # ~ "angular_vel_control_left_front_wheel," +
                  # ~ "Kp*error_curr," +
                  # ~ "Ki*integral," +
                  # ~ "Kd*derivative," +
                  # ~ "control_signal," +
                  # ~ "integral_component_(e*dt)," +
                  # ~ "dt" +
                  # ~ "\n"
                  # ~ )
    # ~ self.left_back_wheel_stats_file =  open("/home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_base_controller/wheels_calibration/left_back_wheel_stats.csv", 'w')
    # ~ self.left_back_wheel_stats_file.write(
                  # ~ "vel_target_lineal," +
                  # ~ "vel_target_angular," +
                  # ~ "angular_vel_target_left_back_wheel," +
                  # ~ "angular_vel_encoder_left_back_wheel," +
                  # ~ "angular_vel_control_left_back_wheel," +
                  # ~ "Kp*error_curr," +
                  # ~ "Ki*integral," +
                  # ~ "Kd*derivative," +
                  # ~ "control_signal," +
                  # ~ "integral_component_(e*dt)," +
                  # ~ "dt" +
                  # ~ "\n"
                  # ~ )
    # ~ self.right_front_wheel_stats_file = open("/home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_base_controller/wheels_calibration/right_front_wheel_stats.csv", 'w')
    # ~ self.right_front_wheel_stats_file.write(
                  # ~ "vel_target_lineal," +
                  # ~ "vel_target_angular," +
                  # ~ "angular_vel_target_right_front_wheel," +
                  # ~ "angular_vel_encoder_right_front_wheel," +
                  # ~ "angular_vel_control_right_front_wheel," +
                  # ~ "Kp*error_curr," +
                  # ~ "Ki*integral," +
                  # ~ "Kd*derivative," +
                  # ~ "control_signal," +
                  # ~ "integral_component_(e*dt)," +
                  # ~ "dt" +
                  # ~ "\n"
                  # ~ )
    # ~ self.right_back_wheel_stats_file = open("/home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_base_controller/wheels_calibration/right_back_wheel_stats.csv", 'w')
    # ~ self.right_back_wheel_stats_file.write(
                  # ~ "vel_target_lineal," +
                  # ~ "vel_target_angular," +
                  # ~ "angular_vel_target_right_back_wheel," +
                  # ~ "angular_vel_encoder_right_back_wheel," +
                  # ~ "angular_vel_control_right_back_wheel," +
                  # ~ "Kp*error_curr," +
                  # ~ "Ki*integral," +
                  # ~ "Kd*derivative," +
                  # ~ "control_signal," +
                  # ~ "integral_component_(e*dt)," +
                  # ~ "dt" +
                  # ~ "\n"
                  # ~ )

    while not rospy.is_shutdown():
      time_diff_update = (time_curr_update - self.time_prev_update).to_sec()
      if time_diff_update < self.timeout_idle: # Only move if command given recently
        self.rwheels_update()
        self.lwheels_update()
      rate.sleep()
    rospy.spin();



  def shutdown(self):
    rospy.loginfo("vva_base_controller: Stop. CHAO PESCAO!")

    ## DEBUG
    # ~ self.left_front_wheel_stats_file.close()
    # ~ self.left_back_wheel_stats_file.close()
    # ~ self.right_front_wheel_stats_file.close()
    # ~ self.right_back_wheel_stats_file.close()

    # Stop message
    rospy.sleep(1)    



  # ==================================================
  # Wheels update functions
  # ==================================================
  # Suppose we have a target velocity v and angular velocity w
  # Suppose we have a robot with only two wheels (right and left) of radius R and distance between wheels L
  # Let vr and vl be angular wheel velocity for right and left wheels, respectively
  # Relate 2v = R (vr +vl) because the forward speed is the sum of the combined wheel velocities
  # Relate Lw = R (vr - vl) because rotation is a function of counter-clockwise wheel speeds
  # Compute vr = (2v + wL) / 2R
  # Compute vl = (2v - wL) / 2R

  # Left wheels
  def lwheels_update(self):
    angular_vel_left_wheels_target = (2*self.target_v - self.target_w*self.L) / self.wheel_D
    
    angular_vel_left_front_wheel_control = 0
    angular_vel_left_back_wheel_control = 0
    # If we want to adjust target angular velocity using PID controller to incorporate encoder readings
    if self.pid_on:
      if self.target_v == 0 and self.target_w != 0:
        Kp = self.rotation_Kp
        Ki = self.rotation_Ki
        Kd = self.rotation_Kd
      else:
        Kp = self.translation_Kp
        Ki = self.translation_Ki
        Kd = self.translation_Kd
      
      angular_vel_left_front_wheel_control, f_control_signal = self.pid_control(self.lfwheel_pid, angular_vel_left_wheels_target,
                                                            self.angular_vel_left_front_wheel_enc, Kp, Ki, Kd)
      angular_vel_left_back_wheel_control, b_control_signal = self.pid_control(self.lbwheel_pid, angular_vel_left_wheels_target,
                                                           self.angular_vel_left_back_wheel_enc, Kp, Ki, Kd)
    else:
      angular_vel_left_front_wheel_control = angular_vel_left_wheels_target
      angular_vel_left_back_wheel_control = angular_vel_left_wheels_target
      
    # Publish the angular velocities of control so the Arduino can move the motors
    self.vva_angular_vel_left_front_wheel_control_pub.publish(angular_vel_left_front_wheel_control)
    self.vva_angular_vel_left_back_wheel_control_pub.publish(angular_vel_left_back_wheel_control)
      
    
    ## DEBUG
    # ~ if self.target_v != 0 or self.target_w != 0:
      # ~ rospy.loginfo("angular_vel_target_left_wheels: %f", angular_vel_left_wheels_target)
      # ~ rospy.loginfo("angular vel encoder left_front_wheel: %f", self.angular_vel_left_front_wheel_enc)
      # ~ rospy.loginfo("angular vel encoder left_back_wheel: %f", self.angular_vel_left_back_wheel_enc)
      # ~ rospy.loginfo("angular vel control left_front_wheel: %f", angular_vel_left_front_wheel_control)
      # ~ rospy.loginfo("angular vel control left_back_wheel: %f", angular_vel_left_back_wheel_control)
    # ~ self.left_front_wheel_stats_file.write("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n" % (
                                     # ~ self.target_v,
                                     # ~ self.target_w,
                                     # ~ angular_vel_left_wheels_target,
                                     # ~ self.angular_vel_left_front_wheel_enc,
                                     # ~ angular_vel_left_front_wheel_control,
                                     # ~ Kp*self.lfwheel_pid['error_curr'],
                                     # ~ Ki*sum(self.lfwheel_pid['integral']),
                                     # ~ Kd*self.lfwheel_pid['derivative'],
                                     # ~ f_control_signal,
                                     # ~ self.lfwheel_pid['integral'][-1],
                                     # ~ self.lfwheel_pid['dt']
                                     # ~ ) )
    # ~ self.left_back_wheel_stats_file.write("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n" % (
                                     # ~ self.target_v,
                                     # ~ self.target_w,
                                     # ~ angular_vel_left_wheels_target,
                                     # ~ self.angular_vel_left_back_wheel_enc,
                                     # ~ angular_vel_left_back_wheel_control,
                                     # ~ Kp*self.lbwheel_pid['error_curr'],
                                     # ~ Ki*sum(self.lbwheel_pid['integral']),
                                     # ~ Kd*self.lbwheel_pid['derivative'],
                                     # ~ b_control_signal,
                                     # ~ self.lbwheel_pid['integral'][-1],
                                     # ~ self.lbwheel_pid['dt']
                                     # ~ ) )


  # Right wheels
  def rwheels_update(self):
    angular_vel_right_wheels_target = (2*self.target_v + self.target_w*self.L) / self.wheel_D

    angular_vel_right_front_wheel_control =0
    angular_vel_right_back_wheel_control =0
    # If we want to adjust target angular velocity using PID controller to incorporate encoder readings
    if self.pid_on:
      if self.target_v == 0 and self.target_w != 0:
        Kp = self.rotation_Kp
        Ki = self.rotation_Ki
        Kd = self.rotation_Kd
      else:
        Kp = self.translation_Kp
        Ki = self.translation_Ki
        Kd = self.translation_Kd

      angular_vel_right_front_wheel_control, f_control_signal = self.pid_control(self.rfwheel_pid, angular_vel_right_wheels_target,
                                                             self.angular_vel_right_front_wheel_enc, Kp, Ki, Kd)
      angular_vel_right_back_wheel_control, b_control_signal = self.pid_control(self.rbwheel_pid, angular_vel_right_wheels_target,
                                                            self.angular_vel_right_back_wheel_enc, Kp, Ki, Kd)
    else:
      angular_vel_right_front_wheel_control = angular_vel_right_wheels_target
      angular_vel_right_back_wheel_control = angular_vel_right_wheels_target

    # Publish the angular velocities of control so the Arduino can move the motors
    self.vva_angular_vel_right_front_wheel_control_pub.publish(angular_vel_right_front_wheel_control)
    self.vva_angular_vel_right_back_wheel_control_pub.publish(angular_vel_right_back_wheel_control)


    ## DEBUG
    # ~ if self.target_v != 0 or self.target_w != 0:
      # ~ rospy.loginfo("---------------------------")
      # ~ rospy.loginfo("vel_target_lineal: %f", self.target_v)
      # ~ rospy.loginfo("vel_target_angular: %f", self.target_w)
      # ~ rospy.loginfo("angular_vel_target_right_wheels: %f", angular_vel_right_wheels_target)
      # ~ rospy.loginfo("angular_vel_encoder_right_front_wheel: %f", self.angular_vel_right_front_wheel_enc)
      # ~ rospy.loginfo("angular_vel_encoder_right_back_wheel: %f", self.angular_vel_right_back_wheel_enc)
      # ~ rospy.loginfo("angular_vel_control_right_front_wheel: %f", angular_vel_right_front_wheel_control)
      # ~ rospy.loginfo("angular_vel_control_right_back_wheel: %f", angular_vel_right_back_wheel_control)
    # ~ self.right_front_wheel_stats_file.write("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n" % (
                                      # ~ self.target_v,
                                      # ~ self.target_w,
                                      # ~ angular_vel_right_wheels_target,
                                      # ~ self.angular_vel_right_front_wheel_enc,
                                      # ~ angular_vel_right_front_wheel_control,
                                      # ~ Kp*self.rfwheel_pid['error_curr'],
                                      # ~ Ki*sum(self.rfwheel_pid['integral']),
                                      # ~ Kd*self.rfwheel_pid['derivative'],
                                      # ~ f_control_signal,
                                      # ~ self.rfwheel_pid['integral'][-1],
                                      # ~ self.rfwheel_pid['dt']
                                      # ~ ) )
    # ~ self.right_back_wheel_stats_file.write("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n" % (
                                      # ~ self.target_v,
                                      # ~ self.target_w,
                                      # ~ angular_vel_right_wheels_target,
                                      # ~ self.angular_vel_right_back_wheel_enc,
                                      # ~ angular_vel_right_back_wheel_control,
                                      # ~ Kp*self.rbwheel_pid['error_curr'],
                                      # ~ Ki*sum(self.rbwheel_pid['integral']),
                                      # ~ Kd*self.rbwheel_pid['derivative'],
                                      # ~ b_control_signal,
                                      # ~ self.rbwheel_pid['integral'][-1],
                                      # ~ self.rbwheel_pid['dt']
                                      # ~ ) )



  # ==================================================
  # PID control
  # ==================================================
  def pid_control(self, wheel_pid, target, state, Kp, Ki, Kd):
    # Initialize pid dictionary
    if len(wheel_pid) == 0:
      wheel_pid.update({'time_prev':rospy.Time.now(), 'derivative':0, 'integral':[0]*10, 'error_prev':0, 'error_curr':0})

    wheel_pid['time_curr'] = rospy.Time.now()

    # PID control
    wheel_pid['dt'] = (wheel_pid['time_curr'] - wheel_pid['time_prev']).to_sec()
    if wheel_pid['dt'] == 0: return target, 0

    wheel_pid['error_curr'] = target - state
    wheel_pid['integral'] = wheel_pid['integral'][1:] + [(wheel_pid['error_curr']*wheel_pid['dt'])]
    wheel_pid['derivative'] = (wheel_pid['error_curr'] - wheel_pid['error_prev'])/wheel_pid['dt']
    wheel_pid['error_prev'] = wheel_pid['error_curr']
    
    control_signal = (Kp*wheel_pid['error_curr'] + Ki*sum(wheel_pid['integral']) - Kd*wheel_pid['derivative'])
    
    target_new = target + control_signal

    # Ensure control_signal does not flip sign of target velocity
    if target > 0 and target_new < 0: target_new = target;
    if target < 0 and target_new > 0: target_new = target;

    if (target == 0): # Not moving
      target_new = 0

    wheel_pid['time_prev'] = wheel_pid['time_curr']
    return target_new, control_signal


        
  # ==================================================
  # Read the subscribed topics
  # ==================================================
  def twistCallback(self,msg):
    self.target_v = msg.linear.x;
    self.target_w = msg.angular.z;
    self.time_prev_update = rospy.Time.now()
    
  def angular_vel_left_front_wheel_enc_callback(self, msg):
    self.angular_vel_left_front_wheel_enc = msg.data
    
  def angular_vel_left_back_wheel_enc_callback(self, msg):
    self.angular_vel_left_back_wheel_enc = msg.data
    
  def angular_vel_right_front_wheel_enc_callback(self, msg):
    self.angular_vel_right_front_wheel_enc = msg.data
    
  def angular_vel_right_back_wheel_enc_callback(self, msg):
    self.angular_vel_right_back_wheel_enc = msg.data

  def motor_calibration_params_callback(self, msg):
    rospy.loginfo("vva_base_controller: Wheels calibration parameters generated and received.")
    self.wheels_calibration_params['motor_cmd_max'] = msg.motor_cmd_max
    self.wheels_calibration_params['motor_cmd_min'] = msg.motor_cmd_min
    self.wheels_calibration_params['motor_max_angular_vel'] = msg.motor_max_angular_vel
    self.wheels_calibration_params['motor_min_angular_vel'] = msg.motor_min_angular_vel
    
    rospy.loginfo("vva_base_controller: Calibration parameters:")
    rospy.loginfo("vva_base_controller: motor_cmd_max: %d", self.wheels_calibration_params['motor_cmd_max'])
    rospy.loginfo("vva_base_controller: motor_cmd_min: %d", self.wheels_calibration_params['motor_cmd_min'])
    rospy.loginfo("vva_base_controller: motor_max_angular_vel: %.2f", self.wheels_calibration_params['motor_max_angular_vel'])
    rospy.loginfo("vva_base_controller: motor_min_angular_vel: %.2f", self.wheels_calibration_params['motor_min_angular_vel'])
    
    # Calibration params are stored in a file
    with open(self.calibration_file_path, 'w') as fc:
      pickle.dump(self.wheels_calibration_params, fc)
    rospy.loginfo("vva_base_controller: Calibration completed. File created at: %s", self.calibration_file_path)

  def vva_arduino_status_callback(self, msg):
    self.arduino_status = msg.data


    
# ==================================================
# Main
# ==================================================
def main():
  cmdvel_to_motors = CmdVelToMotorsCommand();
  cmdvel_to_motors.spin()


if __name__ == '__main__':
  main(); 

