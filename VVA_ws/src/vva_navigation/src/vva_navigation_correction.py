#!/usr/bin/env python

#------------------------------------------------------------------------------
# Copyright (c) 2020, Andres A. <andres.arboleda AT gmail DOT com>
# All rights reserved.
#
# Distributed under BSD license.
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
# Modified by:
# andres.arboleda AT gmail DOT com, (nov/2020)
#------------------------------------------------------------------------------


import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import actionlib
import move_base_msgs.msg
from std_srvs.srv import Empty
from vva_msgs.msg import OdomCorrectionStatus
from vva_msgs.msg import NavCorrectionStatus



# Intermediate module between a goal client and move_base, in charge of implementing improvement actions when
# a goal fails. For example, put an intermediate closer goal which is in the same path of the original goal
# to unstuck the robot
class NavigationCorrection:
  def __init__(self):
    rospy.init_node('vva_navigation_correction')
    
    # Subscribed topics
    self.simple_goal_sub =  rospy.Subscriber('vva_navigation_simple/goal', PoseStamped, self.simple_goal_callback)
    self.global_path_sub =  rospy.Subscriber('move_base/GlobalPlanner/plan',  Path, self.global_path_callback)
    self.odom_c_status_sub =  rospy.Subscriber('vva_odom_correction/status',  OdomCorrectionStatus, self.odom_c_status_callback)
    
    # Published topics
    self.nav_corr_status_pub = rospy.Publisher('vva_navigation_correction/status', NavCorrectionStatus, queue_size=10)
    
    # Published services
    self.cancel_goal_srv     = rospy.Service('~cancel_goal', Empty, self.cancel_goal_srv_callback)

    # Parameters
    self.rate                        = rospy.get_param('~rate', 10)
    self.closer_goal_path_poses      = rospy.get_param('~closer_goal_path_poses', 40)
    self.clear_costmaps_service_name = rospy.get_param('~clear_costmaps_service_name', "/move_base/clear_costmaps")
    self.use_odom_correction_status  = rospy.get_param('~use_odom_correction_status', True)
    self.using_hw_rplidar            = rospy.get_param('~using_hw_rplidar', True)
    self.stop_lidar_service_name     = rospy.get_param('~stop_lidar_service_name', "/stop_motor")
    self.start_lidar_service_name    = rospy.get_param('~start_lidar_service_name', "/start_motor")
    self.rplidar_idle_time           = rospy.get_param('~rplidar_idle_time', 30)
    
    # Global variables for subscribed topics
    self.original_simple_goal = None  # Type: geometry_msgs/PoseStamped
    self.global_path = None           # Type: geometry_msgs/PoseStamped[]
    self.odom_c_status = OdomCorrectionStatus.BUSY
    
    # Other global variables
    self.last_original_simple_goal_stamp = None
    self.nav_corr_status = actionlib.GoalStatus.PENDING
    self.cancel_goal_pending_to_attend = False
    self.move_base_action_client = None
    self.is_rplidar_stopped = False
    self.rplidar_idle_start_time = rospy.Time.now()

    

  # ==================================================
  # Topics and services callback functions
  # ==================================================
  def simple_goal_callback(self,msg):
    self.original_simple_goal = msg
  
  def global_path_callback(self,msg):
    self.global_path = msg.poses

  def odom_c_status_callback(self,msg):
    self.odom_c_status = msg.status
    
  def cancel_goal_srv_callback(self,req):
    # If goal is in progress in the move_base ActionServer
    if self.move_base_action_client.get_state() == actionlib.GoalStatus.ACTIVE:
      # Send a cancel goal request to the ActionServer
      self.move_base_action_client.cancel_goal()
    # If goal is in internal processing and not sent yet to the ActionServer
    else:  
      # Set the cancel goal flag
      self.cancel_goal_pending_to_attend = True
    return []
    
    
    
    
  # ==================================================
  # Update function
  # ==================================================
  def update(self):
    
    # Check if we received a new goal
    is_new_goal = False
    if self.original_simple_goal != None:
      if self.last_original_simple_goal_stamp == None:
        is_new_goal = True
        self.last_original_simple_goal_stamp = self.original_simple_goal.header.stamp
      else:  
        if self.original_simple_goal.header.stamp > self.last_original_simple_goal_stamp:
          is_new_goal = True
          self.last_original_simple_goal_stamp = self.original_simple_goal.header.stamp
        elif self.original_simple_goal.header.stamp < self.last_original_simple_goal_stamp:
          rospy.loginfo("vva_navigation_correction: Requested goal ignored. Timestamp is older than previously sent goal.")
      
    # If a new goal was received, process and send it to move_base
    if is_new_goal:
      
      # Publish status
      self.nav_corr_status = actionlib.GoalStatus.ACTIVE
      goal_status_message = NavCorrectionStatus()
      goal_status_message.header.stamp = rospy.Time.now()
      goal_status_message.status = self.nav_corr_status
      self.nav_corr_status_pub.publish(goal_status_message)
      
      # If RPLidar is stopped, start it
      if self.is_rplidar_stopped and self.using_hw_rplidar:
        # Start the RPLidar motor
        try:
          rospy.loginfo("vva_navigation_correction: Starting the motor of RPLidar")
          srv_start_rplidar_motor = rospy.ServiceProxy(self.start_lidar_service_name, Empty)
          srv_start_rplidar_motor()
          self.is_rplidar_stopped = False
          # ~ rospy.loginfo("vva_navigation_correction: Waiting 3 seconds...")
          # ~ rospy.sleep(3)
        except rospy.ServiceException as e:
          rospy.logwarn("vva_navigation_correction: Service failed: " + str(e))
      
      rospy.loginfo("vva_navigation_correction: Sending originally requested goal to move_base...")
      goal_result = self.send_simple_goal(self.original_simple_goal)

      # Hold until goal is completed or failed
      
      rospy.loginfo("vva_navigation_correction: Originally requested goal result: %s", self.goal_status2text(goal_result))

      if goal_result == actionlib.GoalStatus.ABORTED:

        # Get a new goal, closer than the original one:
        # The distance between poses of the Global Path is 0.025 meters,
        # except for the distance between the first two and the last 3 poses which may vary.
        
        global_path_lenght = len(self.global_path)
        
        if global_path_lenght > self.closer_goal_path_poses:
          
          # Publish status
          self.nav_corr_status_pub.publish(goal_status_message)

          # Send a closer goal
          closer_goal = self.global_path[self.closer_goal_path_poses]
          rospy.loginfo("vva_navigation_correction: Sending a closer goal to move_base...")
          goal_result = self.send_simple_goal(closer_goal)
          
          # Hold until goal is completed or failed
          
          rospy.loginfo("vva_navigation_correction: Closer goal result: %s", self.goal_status2text(goal_result))
          
          if goal_result == actionlib.GoalStatus.SUCCEEDED:
            # Send again the original goal.
            # Update the original_simple_goal timestamp so it will be sent again
            self.original_simple_goal.header.stamp = rospy.Time.now()
          else:
            self.nav_corr_status = goal_result
      
        else:
          rospy.logwarn('vva_navigation_correction: Could not find a closer goal to unstuck the robot. The global path is shorter than the configured "closer_goal_path_poses".')
          rospy.logwarn('vva_navigation_correction: closer_goal_path_poses: %d, global path lenght: %d ', self.closer_goal_path_poses, global_path_lenght)
          self.nav_corr_status = goal_result
          
      else:
        self.nav_corr_status = goal_result
        
      # Start counting the time to put RPLidar in idle state (stop motor)
      self.rplidar_idle_start_time = rospy.Time.now()
      
        
    # If there is no any new goal to process, count idle time to stop the RPLidar motor
    elif not self.is_rplidar_stopped and self.using_hw_rplidar:
      if (rospy.Time.now() - self.rplidar_idle_start_time).to_sec() >= self.rplidar_idle_time:
        # Stop the RPLidar motor
        try:
          rospy.loginfo("vva_navigation_correction: Stopping the motor of RPLidar")
          srv_stop_rplidar_motor = rospy.ServiceProxy(self.stop_lidar_service_name, Empty)
          srv_stop_rplidar_motor()
          self.is_rplidar_stopped = True
        except rospy.ServiceException as e:
          rospy.logwarn("vva_navigation_correction: Service failed: " + str(e))
        

    # Publish status
    goal_status_message = NavCorrectionStatus()
    goal_status_message.header.stamp = rospy.Time.now()
    goal_status_message.status = self.nav_corr_status
    self.nav_corr_status_pub.publish(goal_status_message)



  # ==================================================
  # Send simple goal
  #  Desc: uses actionlib to send a goal to move_base,
  #    this function pauses the main thread of the node
  #    until the goal is either reached or failed.
  #  Attrib:
  #    simple_goal - Type: geometry_msgs/PoseStamped
  # ==================================================
  def send_simple_goal(self, simple_goal):
    
    if self.use_odom_correction_status:
      # Wait for vva_odom_correction to finish the odometry restart procedures
      rospy.loginfo('vva_navigation_correction: Waiting for "vva_odom_correction" to finish the odometry restart procedures...')
      while self.odom_c_status == OdomCorrectionStatus.BUSY:
        rospy.sleep(0.5)
    
    # Clear the costmaps and wait 2 seconds before sending the goal
    try:
      rospy.loginfo("vva_navigation_correction: Clearing costmaps")
      srv_clear_costmaps = rospy.ServiceProxy(self.clear_costmaps_service_name, Empty)
      srv_clear_costmaps()
      rospy.loginfo("vva_navigation_correction: Waiting 2 seconds...")
      rospy.sleep(2)
    except rospy.ServiceException as e:
      rospy.logwarn("vva_navigation_correction: Service failed: " + str(e))

    # Check if a cancel goal request was received
    if self.cancel_goal_pending_to_attend:
      rospy.loginfo('vva_navigation_correction: Goal cancelled.')
      self.cancel_goal_pending_to_attend = False
      return actionlib.GoalStatus.RECALLED

    # Send the goal to the move_base ActionServer
    goal = move_base_msgs.msg.MoveBaseGoal()
    goal.target_pose.header.frame_id = simple_goal.header.frame_id
    goal.target_pose.header.stamp = simple_goal.header.stamp
    goal.target_pose.pose = simple_goal.pose
  
    rospy.loginfo('vva_navigation_correction: Sending goal (x=%.2f, y=%.2f, rot_z=%.2f, rot_w=%.2f) in "%s" frame.', goal.target_pose.pose.position.x,
             goal.target_pose.pose.position.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w,
             goal.target_pose.header.frame_id)
    self.move_base_action_client.send_goal(goal)

    self.move_base_action_client.wait_for_result()
    
    return self.move_base_action_client.get_state()
    
    
  # ==================================================
  # Translate goal status code to text
  # ==================================================
  def goal_status2text(self, goal_result):
    if goal_result == actionlib.GoalStatus.PENDING:
      return "Pending"
    if goal_result == actionlib.GoalStatus.ACTIVE:
      return "Active"
    if goal_result == actionlib.GoalStatus.ABORTED:
      return "Aborted"
    if goal_result == actionlib.GoalStatus.SUCCEEDED:
      return "Succeeded"
      
    return str(goal_result)


  # ==================================================
  # Main loop
  # ==================================================
  def spin(self):
    rospy.loginfo("vva_navigation_correction: Start")
    rate = rospy.Rate(self.rate)
    rospy.on_shutdown(self.shutdown)
    
    # Initialize the ActionClient for move_base
    self.move_base_action_client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
    rospy.loginfo("vva_navigation_correction: Waiting for move_base ActionServer to connect...")
    self.move_base_action_client.wait_for_server()
    rospy.loginfo("vva_navigation_correction: move_base ActionServer connected")
    
    if self.using_hw_rplidar:
      rospy.loginfo("vva_navigation_correction: Waiting for %s service to be started...", self.stop_lidar_service_name)
      rospy.wait_for_service(self.stop_lidar_service_name)
      rospy.loginfo("vva_navigation_correction: %s service started.", self.stop_lidar_service_name)

    while not rospy.is_shutdown():
      self.update()
      rate.sleep()
    rospy.spin()


  # ==================================================
  # If shutdown
  # ==================================================

  def shutdown(self):
    rospy.loginfo("vva_navigation_correction: Stop. CHAO PESCAO!")
    # Stop message
    rospy.sleep(1)
    

# ==================================================
# Main
# ==================================================
def main():
  navigation_correction = NavigationCorrection()
  navigation_correction.spin()

if __name__ == '__main__':
  main()





