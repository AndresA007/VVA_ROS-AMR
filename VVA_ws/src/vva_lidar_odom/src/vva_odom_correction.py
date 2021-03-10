#!/usr/bin/python

#------------------------------------------------------------------------------
# Copyright (c) 2020, Andres A. <andres.arboleda AT gmail DOT com>
# All rights reserved.
#
# Distributed under BSD license
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
# andres.arboleda AT gmail DOT com, (jun/2020)
#------------------------------------------------------------------------------


import math
import rospy
from std_msgs.msg import Float32
from vva_msgs.msg import OdomCorrectionStatus
from std_srvs.srv import Empty
import tf
from tf2_ros import TransformException
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray


# Periodically restart odometry to identity transformation, to reduce the drift of the costmap
# when the vehicle rotates in place, also to reduce the odom displacement when the vehicle is stopped.
class PeriodicOdomReset:
  def __init__(self):
    rospy.init_node('vva_odom_correction')
    
    # Instantiate a TF listener to transform coordinates between frames
    self.tf_listener = tf.TransformListener()
    
    # Read in angular velocity from wheels encoders
    self.angular_vel_left_wheel_enc_sub =  rospy.Subscriber('vva_angular_vel_left_wheel_enc',  Float32, self.enc_left_wheel_w_callback)
    self.angular_vel_right_wheel_enc_sub = rospy.Subscriber('vva_angular_vel_right_wheel_enc', Float32, self.enc_right_wheel_w_callback)
    self.nav_goal_status_sub = rospy.Subscriber('move_base/status', GoalStatusArray, self.nav_goal_status_callback)
    
    self.map_initialpose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=10)
    self.odom_correction_status_pub = rospy.Publisher('vva_odom_correction/status', OdomCorrectionStatus, queue_size=10)

    self.rate = rospy.get_param('~rate', 10)
    self.reset_odom_service_name = rospy.get_param('~reset_odom_service_name', "/reset_odom")
    # ~ self.clear_costmaps_service_name = rospy.get_param('~clear_costmaps_service_name', "/move_base/clear_costmaps")
    self.reset_odom_period_idle = rospy.get_param('~reset_odom_period_idle', 120)
    self.reset_odom_period_moving = rospy.get_param('~reset_odom_period_moving', 5)
    self.busy_time_before_odom_reset = rospy.get_param('~busy_time_before_odom_reset', 2)
    self.map_frame = rospy.get_param('~map_frame', "map")
    self.base_frame = rospy.get_param('~base_frame', "base_link")
    
    self.update_time = rospy.Time.now()
    self.reset_odom_period = self.reset_odom_period_idle
    self.prev_wheels_stopped = True
    self.is_odometry_aligned = True
    # Flag to indicate that the odometry is in resetting process and no goals should be requested
    self.odom_correction_status = OdomCorrectionStatus.IDLE
    
    # Since the localization node (rtabmap) sometimes ignores the initialpose topic, we need a retry mechanism
    self.initialpose_confirm_pending = False
    self.last_initial_pose = None
    
    self.enc_left_wheel_w  = 0
    self.enc_right_wheel_w = 0
    
    # nav_goal_status: -3: move_base not started yet,
    #                  -2: move_base started but no goal has been set yet,
    #                   1: Moving to goal in progress,
    #                   3: Goal reached,
    #                   4: Goal couldn't be reached 
    self.nav_goal_status = -3   

  # ==================================================
  # Read the subscribed topics
  # ==================================================
  def enc_left_wheel_w_callback(self,msg):
    self.enc_left_wheel_w = msg.data

  def enc_right_wheel_w_callback(self,msg):
    self.enc_right_wheel_w = msg.data
    
  def nav_goal_status_callback(self,msg):
    if not msg.status_list:
      self.nav_goal_status = -2
    else:
      self.nav_goal_status = msg.status_list[0].status
      
    
    
  # ==================================================
  # Update function
  # ==================================================
  def update(self):
    
    # If vehicle is stopped
    if self.enc_left_wheel_w == 0 and self.enc_right_wheel_w == 0:
      
      # If wheels were previously moving and just stopped, start counting the period of time
      if not self.prev_wheels_stopped:
        self.reset_odom_period = self.reset_odom_period_moving
        self.update_time = rospy.Time.now()
        self.is_odometry_aligned = False
        self.odom_correction_status = OdomCorrectionStatus.BUSY

      # Publish busy status some time before the period expires, to indicate that no new goals should be assigned
      if (rospy.Time.now() - self.update_time).to_sec() >= (self.reset_odom_period - self.busy_time_before_odom_reset):
        self.odom_correction_status = OdomCorrectionStatus.BUSY

      # If period expired
      if (rospy.Time.now() - self.update_time).to_sec() >= self.reset_odom_period:
        # If there is no any move_base goal in progress, restart the odometry
        if self.nav_goal_status != 1:
          self.reset_odometry()
        else:
          rospy.loginfo("vva_odom_correction: move_base goal in progress odometry restart aborted.")
        # Restart counting the period of time  
        self.update_time = rospy.Time.now()
        self.reset_odom_period = self.reset_odom_period_idle
        self.odom_correction_status = OdomCorrectionStatus.IDLE
        
      self.prev_wheels_stopped = True
      
    # If vehicle is moving
    else:
      self.prev_wheels_stopped = False
      self.odom_correction_status = OdomCorrectionStatus.BUSY
    
    # If there is an initialpose received confirmation pending, resend the topic in each cycle
    if self.initialpose_confirm_pending:
      self.odom_correction_status = OdomCorrectionStatus.BUSY
      self.resend_initialpose()
      # If in resend_initialpose() was detected that the initialpose was successfully accepted, set the status in idle
      if not self.initialpose_confirm_pending:
        self.odom_correction_status = OdomCorrectionStatus.IDLE
    
    # Publish status
    oc_status = OdomCorrectionStatus()
    oc_status.header.stamp = rospy.Time.now()
    oc_status.status = self.odom_correction_status
    self.odom_correction_status_pub.publish(oc_status)
    
      


  # ==================================================
  # Reset odometry function
  # ==================================================
  def reset_odometry(self):
    time_now = rospy.Time.now()
    reset_position_in_map = True
         
    # If odometry is aligned with the base, don't set position in the map
    if self.is_odometry_aligned:
      reset_position_in_map = False
         
    rospy.loginfo("vva_odom_correction: Are odom and base frames aligned: " + str(self.is_odometry_aligned))
  
    if reset_position_in_map:
      # Get current position of the robot in the map
      try:
        # ~ rospy.loginfo("vva_odom_correction: Waiting for transform %s --> %s...", self.map_frame, self.base_frame)
        self.tf_listener.waitForTransform(self.map_frame, self.base_frame, time_now, rospy.Duration(4.0))
        (map_base_trans, map_base_rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, time_now)
        # ~ rospy.loginfo("vva_odom_correction: Transform got, current robot position in the map x: %.2f, y: %.2f, approx_rot: %.2f",
                      # ~ map_base_trans[0], map_base_trans[1], map_base_rot[2])
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, TransformException) as e:
        rospy.logwarn("vva_odom_correction: Frame transform (TF) exception: " + str(e))
        reset_position_in_map = False


    rospy.loginfo("vva_odom_correction: Reset position in the map frame: " + str(reset_position_in_map))

    
    # Restart odometry
    try:
      
      # TODO: Test stopping the odometry beofre reset and restoring after /initialpose is set
      
      rospy.loginfo("vva_odom_correction: Restarting odometry")
      srv_reset_odom = rospy.ServiceProxy(self.reset_odom_service_name, Empty)
      srv_reset_odom()
      self.is_odometry_aligned = True
    except rospy.ServiceException as e:
      rospy.logwarn("vva_odom_correction: Service failed: " + str(e))
      
    if reset_position_in_map:
      # Set the base position in the map, where it was before the odometry reset.
      rospy.loginfo("vva_odom_correction: Sending initial pose to the map")
      pose = PoseWithCovarianceStamped()
      pose.header.frame_id = self.map_frame
      pose.header.stamp = time_now
      
      pose.pose.pose.position.x = map_base_trans[0]
      pose.pose.pose.position.y = map_base_trans[1]
      pose.pose.pose.position.z = map_base_trans[2]
      
      pose.pose.pose.orientation.x = map_base_rot[0]
      pose.pose.pose.orientation.y = map_base_rot[1]
      pose.pose.pose.orientation.z = map_base_rot[2]
      pose.pose.pose.orientation.w = map_base_rot[3]
      
      pose.pose.covariance[6*0+0] = 0.5 * 0.5
      pose.pose.covariance[6*1+1] = 0.5 * 0.5
      pose.pose.covariance[6*5+5] = math.pi/12.0 * math.pi/12.0
      
      self.map_initialpose_pub.publish(pose)
      
      self.last_initial_pose = (map_base_trans, map_base_rot)
      self.initialpose_confirm_pending = True
      


  # ==================================================
  # Resend initialpose topic
  # ==================================================
  def resend_initialpose(self):
    
    time_now = rospy.Time.now()

    # Check if initial pose was received and processed
    try:
      # Get current position of the robot in the map
      rospy.loginfo("vva_odom_correction: Checking if initialpose was received and processed:")
      # ~ rospy.loginfo("vva_odom_correction: Waiting for transform %s --> %s...", self.map_frame, self.base_frame)
      self.tf_listener.waitForTransform(self.map_frame, self.base_frame, time_now, rospy.Duration(4.0))
      (map_base_trans, map_base_rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, time_now)
      # ~ rospy.loginfo("vva_odom_correction: Transform got, current robot position in the map x: %.2f, y: %.2f, approx_rot: %.2f",
                    # ~ map_base_trans[0], map_base_trans[1], map_base_rot[2])
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
      rospy.logwarn("vva_odom_correction: Frame transform (TF) exception: " + str(e))
      reset_position_in_map = False


    ## DEBUG
    # ~ rospy.loginfo("vva_odom_correction: self.last_initial_pose[0][0]: %.2f, map_base_trans[0]: %.2f", self.last_initial_pose[0][0], map_base_trans[0])
    # ~ rospy.loginfo("vva_odom_correction: self.last_initial_pose[0][1]: %.2f, map_base_trans[1]: %.2f", self.last_initial_pose[0][1], map_base_trans[1])
    # ~ rospy.loginfo("vva_odom_correction: self.last_initial_pose[1][2]: %.2f, map_base_rot[2]: %.2f",   self.last_initial_pose[1][2], map_base_rot[2]  )
      
    # If current position is approximately equal to the published initialpose, then the initialpose is confirmed
    if ( abs(self.last_initial_pose[0][0] - map_base_trans[0]) <= 0.05 and
         abs(self.last_initial_pose[0][1] - map_base_trans[1]) <= 0.05 and
         abs(abs(self.last_initial_pose[1][2]) - abs(map_base_rot[2])) <= 0.05 ):
           
      self.initialpose_confirm_pending = False
      rospy.loginfo("vva_odom_correction: Initial pose successfully set.")
      
      # If move_base is running, clear the costmaps
      # ~ if self.nav_goal_status != -3:
        # ~ try:
          # ~ rospy.loginfo("Clearing costmaps")
          # ~ srv_clear_costmaps = rospy.ServiceProxy(self.clear_costmaps_service_name, Empty)
          # ~ srv_clear_costmaps()
        # ~ except rospy.ServiceException as e:
          # ~ rospy.logwarn("Service failed: " + str(e))
      
    else:
      rospy.loginfo("vva_odom_correction: Re-sending initial pose to the map")
      
      (last_map_base_trans, last_map_base_rot) = self.last_initial_pose
      
      pose = PoseWithCovarianceStamped()
      pose.header.frame_id = self.map_frame
      pose.header.stamp = time_now
      
      pose.pose.pose.position.x = last_map_base_trans[0]
      pose.pose.pose.position.y = last_map_base_trans[1]
      pose.pose.pose.position.z = last_map_base_trans[2]
      
      pose.pose.pose.orientation.x = last_map_base_rot[0]
      pose.pose.pose.orientation.y = last_map_base_rot[1]
      pose.pose.pose.orientation.z = last_map_base_rot[2]
      pose.pose.pose.orientation.w = last_map_base_rot[3]
      
      pose.pose.covariance[6*0+0] = 0.5 * 0.5
      pose.pose.covariance[6*1+1] = 0.5 * 0.5
      pose.pose.covariance[6*5+5] = math.pi/12.0 * math.pi/12.0
      
      self.map_initialpose_pub.publish(pose)
    



  # ==================================================
  # Main loop
  # ==================================================
  def spin(self):
    rospy.loginfo("vva_odom_correction: Start")
    rate = rospy.Rate(self.rate)
    rospy.on_shutdown(self.shutdown)
    
    rospy.loginfo("vva_odom_correction: Waiting for %s service to be started...", self.reset_odom_service_name)
    rospy.wait_for_service(self.reset_odom_service_name)
    rospy.loginfo("vva_odom_correction: %s service started.", self.reset_odom_service_name)

    while not rospy.is_shutdown():
      self.update();
      rate.sleep()
    rospy.spin()


  # ==================================================
  # If shutdown
  # ==================================================

  def shutdown(self):
    rospy.loginfo("vva_odom_correction: Stop. CHAO PESCAO!")
    # Stop message
    rospy.sleep(1)
    

# ==================================================
# Main
# ==================================================
def main():
  periodic_odom_reset = PeriodicOdomReset();
  periodic_odom_reset.spin()

if __name__ == '__main__':
  main(); 





