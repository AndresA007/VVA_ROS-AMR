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
# andres.arboleda AT gmail DOT com
#------------------------------------------------------------------------------

import rospy

# Messages
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class VVATopicAnalyzer:
  def __init__(self):
    rospy.init_node('vva_topic_analyzer_accel_limits')
    self.cmdvel_sub = rospy.Subscriber('cmd_vel', Twist, self.twistCallback)
    
    self.odom_sub = rospy.Subscriber('/scanmatch_odom', Odometry, self.odometry_callback)

    self.vva_angular_vel_left_front_wheel_enc_sub = rospy.Subscriber('vva_angular_vel_left_front_wheel_enc',
                                                     Float32, self.angular_vel_left_front_wheel_enc_callback)
    self.vva_angular_vel_left_back_wheel_enc_sub = rospy.Subscriber('vva_angular_vel_left_back_wheel_enc',
                                                     Float32, self.angular_vel_left_back_wheel_enc_callback)
    self.vva_angular_vel_right_front_wheel_enc_sub = rospy.Subscriber('vva_angular_vel_right_front_wheel_enc',
                                                     Float32, self.angular_vel_right_front_wheel_enc_callback)
    self.vva_angular_vel_right_back_wheel_enc_sub = rospy.Subscriber('vva_angular_vel_right_back_wheel_enc',
                                                     Float32, self.angular_vel_right_back_wheel_enc_callback)

    # Update the dimensions parameters of the vehicle
    self.L = rospy.get_param('~robot_wheel_separation_distance', 0.38)
    self.wheel_D = rospy.get_param('~robot_wheel_diameter', 0.13)
    self.rate = rospy.get_param('~rate', 10)
    
    # Target vels:
    self.target_lineal_x = 0
    self.target_lineal_y = 0
    self.target_w = 0
    
    # Odom vels:
    self.odom_lineal_x = 0
    self.odom_lineal_y = 0
    self.odom_w = 0

    # Wheels encoders vels:
    self.angular_vel_left_front_wheel_enc = 0
    self.angular_vel_left_back_wheel_enc = 0
    self.angular_vel_right_front_wheel_enc = 0
    self.angular_vel_right_back_wheel_enc = 0
    
    # To find the acceleration limits based on odometry
    self.stopped_state = 1
    self.starting_to_move_state = 2
    self.moving_accelm_pending_state = 3
    self.moving_accelm_done_state = 4
    self.stopping_state = 5
    
    self.accel_measure_states_translate = {
      1: 'Stopped.',
      2: 'Starting to move...',
      3: 'Moving, acceleration measurement pending...',
      4: 'Moving, acceleration measurement done.',
      5: 'Stopping...',
    }

    
    self.curr_state = 1
    self.state_prev_cycle = 1
    
    self.measuring_lineal_accel = False
    self.measuring_angular_accel = False
    
    self.orig_target_lineal_x = 0
    self.orig_odom_lineal_x = 0
    self.lineal_accel_start_time = None
    self.orig_target_w = 0
    self.orig_odom_w = 0
    self.angular_accel_start_time = None

    # Statistics file
    # ~ self.vel_stats_file = None
    

  # ==================================================
  # Main loop
  # ==================================================
  # When given no commands for some time, do not update
  def spin(self):
    rospy.loginfo("Start vva_topic_analyzer_accel_limits")
    rate = rospy.Rate(self.rate)
    time_curr_update = rospy.Time.now()
    
    rospy.on_shutdown(self.shutdown)
    
    rospy.loginfo("Vehicle dimensions parameters:")
    rospy.loginfo("robot_wheel_separation_distance: %.2f m", self.L)
    rospy.loginfo("robot_wheel_diameter: %.3f m", self.wheel_D)
    rospy.loginfo("Sampling rate: %.1f Hz", self.rate)
    
    # Write headers in statistics file
    # ~ self.vel_stats_file =  open("/home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_nav_test/topic_analyzer_stats/vel_stats.csv", 'w')
    # ~ self.vel_stats_file.write(
                  # ~ "Tiempo," +
                  # ~ "target_lineal_x," +
                  # ~ "target_lineal_y," +
                  # ~ "target_w," +
                  # ~ "odom_lineal_x," +
                  # ~ "odom_lineal_y," +
                  # ~ "odom_w," +
                  # ~ "enc_lineal_x," +
                  # ~ "enc_lineal_y," +
                  # ~ "enc_w" +
                  # ~ "\n"
                  # ~ )

    while not rospy.is_shutdown():
      self.update()
      rate.sleep()
    rospy.spin();



  # ==================================================
  # If shutdown
  # ==================================================
  def shutdown(self):
    rospy.loginfo("Stop vva_topic_analyzer_accel_limits. CHAO PESCAO!")
    
    # ~ self.vel_stats_file.close()

    # Stop message
    rospy.sleep(1)    


  # ==================================================
  # update function
  # ==================================================
  def update(self):
    
    # ~ if self.target_lineal_x != 0 or self.target_w != 0:
      # ~ enc_lineal_x, enc_lineal_y, enc_w = self.wheel_to_base_vel()
      
      # ~ rospy.loginfo("Tiempo: %f", rospy.Time.now().to_sec())
      # ~ rospy.loginfo("target_lineal_x: %f", self.target_lineal_x)
      # ~ rospy.loginfo("target_lineal_y: %f", self.target_lineal_y)
      # ~ rospy.loginfo("target_w: %f", self.target_w)
      # ~ rospy.loginfo("odom_lineal_x: %f", self.odom_lineal_x)
      # ~ rospy.loginfo("odom_lineal_y: %f", self.odom_lineal_y)
      # ~ rospy.loginfo("odom_w: %f", self.odom_w)
      # ~ rospy.loginfo("enc_lineal_x: %f", enc_lineal_x)
      # ~ rospy.loginfo("enc_lineal_y: %f", enc_lineal_y)
      # ~ rospy.loginfo("enc_w: %f", enc_w)
      
      # ~ self.vel_stats_file.write("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n" % (
                                     # ~ rospy.Time.now().to_sec(),
                                     # ~ self.target_lineal_x,
                                     # ~ self.target_lineal_y,
                                     # ~ self.target_w,
                                     # ~ self.odom_lineal_x,
                                     # ~ self.odom_lineal_y,
                                     # ~ self.odom_w,
                                     # ~ enc_lineal_x,
                                     # ~ enc_lineal_y,
                                     # ~ enc_w
                                     # ~ ) )


      # Find the acceleration limits based on odometry
      
      if self.curr_state != self.state_prev_cycle:
        rospy.loginfo("Current state: %s", self.accel_measure_states_translate[self.curr_state])
        
      # If vehicle is stopped and no /cmd_vel has been received yet
      if (self.target_lineal_x == 0 and self.target_w == 0 and
          self.angular_vel_left_front_wheel_enc  == 0 and self.angular_vel_left_back_wheel_enc  == 0 and
          self.angular_vel_right_front_wheel_enc == 0 and self.angular_vel_right_back_wheel_enc == 0):
            
        self.state_prev_cycle = self.curr_state
        self.curr_state = self.stopped_state
        
      # If vehicle received /cmd_vel but is not moving yet
      elif (self.angular_vel_left_front_wheel_enc  == 0 and self.angular_vel_left_back_wheel_enc  == 0 and
            self.angular_vel_right_front_wheel_enc == 0 and self.angular_vel_right_back_wheel_enc == 0):
              
        self.state_prev_cycle = self.curr_state
        self.curr_state = self.starting_to_move_state
        
        # If this is the first cycle receiving /cmd_vel start counting time either for lineal or angular vel
        if self.state_prev_cycle == self.stopped_state:
          if self.target_lineal_x != 0:
            self.measuring_lineal_accel = True
            self.orig_target_lineal_x = self.target_lineal_x
            self.orig_odom_lineal_x = self.odom_lineal_x
            self.lineal_accel_start_time = rospy.Time.now()
          if self.target_w != 0:
            self.measuring_angular_accel = True
            self.orig_target_w = self.target_w
            self.orig_odom_w = self.odom_w
            self.angular_accel_start_time = rospy.Time.now()
        
      # If vehicle is no longer receiving /cmd_vel but is still moving  
      elif self.target_lineal_x == 0 and self.target_w == 0:
        self.state_prev_cycle = self.curr_state
        self.curr_state = self.stopping_state
        
      # If vehicle is receiving /cmd_vel, is moving (according to the encoders) and has measurement in process
      elif self.measuring_lineal_accel or self.measuring_angular_accel:
        self.state_prev_cycle = self.curr_state
        self.curr_state = self.moving_accelm_pending_state
      
        # If lineal acceleration measurement is in progress
        if self.measuring_lineal_accel:
          # Threshold is 100% of the terget lineal vel
          if abs(self.odom_lineal_x) >= abs(1.0 * self.orig_target_lineal_x):
            self.measuring_lineal_accel = False
            
            lineal_acceleration = (self.odom_lineal_x - self.orig_odom_lineal_x) / (rospy.Time.now() - self.lineal_accel_start_time).to_sec()
            rospy.loginfo("Lineal acceleration (acc_lim_x): %.2f m/s2", lineal_acceleration)
        
        # If angular acceleration measurement is in progress
        if self.measuring_angular_accel:
          # Threshold is 35% of the terget lineal vel, the odom_w almost never reports higher vel. due to friction
          if abs(self.odom_w) >= abs(0.35 * self.orig_target_w):
            self.measuring_angular_accel = False
            
            angular_acceleration = (self.odom_w - self.orig_odom_w) / (rospy.Time.now() - self.angular_accel_start_time).to_sec()
            rospy.loginfo("Angular acceleration (acc_lim_theta): %.2f rad/s2", angular_acceleration)
            
      # If vehicle is receiving /cmd_vel, is moving (according to the encoders) and has measurement was done
      else:
        self.state_prev_cycle = self.curr_state
        self.curr_state = self.moving_accelm_done_state
        


        
  # ==================================================
  # Read the subscribed topics
  # ==================================================
  def twistCallback(self,msg):
    self.target_lineal_x = msg.linear.x
    self.target_lineal_y = msg.linear.y
    self.target_w = msg.angular.z
    
  def odometry_callback(self,msg):
    self.odom_lineal_x = msg.twist.twist.linear.x
    self.odom_lineal_y = msg.twist.twist.linear.y
    self.odom_w = msg.twist.twist.angular.z
  
  def angular_vel_left_front_wheel_enc_callback(self, msg):
    self.angular_vel_left_front_wheel_enc = msg.data
    
  def angular_vel_left_back_wheel_enc_callback(self, msg):
    self.angular_vel_left_back_wheel_enc = msg.data
    
  def angular_vel_right_front_wheel_enc_callback(self, msg):
    self.angular_vel_right_front_wheel_enc = msg.data
    
  def angular_vel_right_back_wheel_enc_callback(self, msg):
    self.angular_vel_right_back_wheel_enc = msg.data

  
  
  
  # ==================================================
  # wheel_to_base_vel: Convert wheel angular vel to base Vx, Vy and w
  # ==================================================
  def wheel_to_base_vel(self):
    # Get the average angular velocity reported by the two encoders of the left and the two of the right
    right_enc_wheel_w = (self.angular_vel_right_front_wheel_enc + self.angular_vel_right_back_wheel_enc) / 2
    left_enc_wheel_w = (self.angular_vel_left_front_wheel_enc + self.angular_vel_left_back_wheel_enc) / 2
    
    base_v = self.wheel_D * (right_enc_wheel_w + left_enc_wheel_w) / 4
    base_w = self.wheel_D * (right_enc_wheel_w - left_enc_wheel_w) / (2 * self.L)
    
    return base_v, 0, base_w
  
  
    
# ==================================================
# Main
# ==================================================
def main():
  vva_topic_analyzer = VVATopicAnalyzer();
  vva_topic_analyzer.spin()


if __name__ == '__main__':
  main(); 






