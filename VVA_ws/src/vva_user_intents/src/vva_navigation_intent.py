#!/usr/bin/env python

#------------------------------------------------------------------------------
# Copyright (c) 2020, Andres A. <andres.arboleda AT gmail DOT com>
# All rights reserved.
# 
# Distributed under the BSD License.
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
# andres.arboleda AT gmail DOT com, (aug/2020)
# Modified by:
# andres.arboleda AT gmail DOT com, (nov/2020)
#------------------------------------------------------------------------------


import rospy
from geometry_msgs.msg import PoseStamped
from vva_msgs.msg import NavCorrectionStatus
from vva_msgs.srv import VVAVoiceCommandIntent
from std_srvs.srv import Empty
import actionlib
import tf
from tf2_ros import TransformException


# Attends service calls from the Speech Recognition node according to the voice
# commands received from the user.
class NavigationIntent:
  def __init__(self):
    rospy.init_node('vva_navigation_intent')
    
    # Subscribed topics
    self.goal_status_sub = rospy.Subscriber('vva_navigation_correction/status', NavCorrectionStatus, self.goal_status_topic_callback)
    
    # Published topics
    self.simple_goal_pub = rospy.Publisher('vva_navigation_simple/goal', PoseStamped, queue_size=10)
    
    # Published services
    self.navigate_to_srv      = rospy.Service('~navigate_to',      VVAVoiceCommandIntent, self.navigate_to_srv_callback)
    self.start_patrolling_srv = rospy.Service('~start_patrolling', VVAVoiceCommandIntent, self.start_patrolling_srv_callback)
    self.stop_navigation_srv  = rospy.Service('~stop_navigation',  VVAVoiceCommandIntent, self.stop_navigation_srv_callback)

    # Parameters
    self.location_names               = rospy.get_param('~location_names')
    self.patrolling_itinerary         = rospy.get_param('~patrolling_itinerary')
    self.rate                         = rospy.get_param('~rate', 10)
    self.stop_navigation_service_name = rospy.get_param('~stop_navigation_service_name', "/vva_navigation_correction/cancel_goal")
    self.map_frame                    = rospy.get_param('~map_frame', "map")
    
    # Global variables for subscribed topics
    self.goal_status = actionlib.GoalStatus.ACTIVE
    
    # Other global variables
    self.navigate_to_pending_to_attend = False
    self.start_patrolling_pending_to_attend = False
    self.stop_navigation_pending_to_attend = False
    
    self.navigate_to_arg = ""
    
    # Instantiate a TF listener to transform coordinates between frames
    self.tf_listener = tf.TransformListener()
    

  # ==================================================
  # Topics and services callback functions
  # ==================================================
  def goal_status_topic_callback(self,msg):
    self.goal_status = msg.status

  def navigate_to_srv_callback(self,req):
    self.navigate_to_pending_to_attend = True
    self.navigate_to_arg = req.intent_service_arg
    return "ACCEPTED_NAVIGATE_TO"
    
  def start_patrolling_srv_callback(self,req):
    self.start_patrolling_pending_to_attend = True
    return "ACCEPTED_START_PATROLLING"
    
  def stop_navigation_srv_callback(self,req):
    self.stop_navigation_pending_to_attend = True
    return "ACCEPTED_STOP_NAVIGATION"
  
  
  # ==================================================
  # Navigate to ...
  # ==================================================
  def navigate_to(self, goal_location_name):

    # Get the coordinates of the goal location
    location_exists, goal_x, goal_y, goal_frame = self.resolve_location(goal_location_name)
    
    if location_exists == False:
      rospy.loginfo('vva_navigation_intent: Location "%s" not found, aborting "navigate to" intent.', goal_location_name)
      return
    
    # Wait for vva_navigation_correction to be ready to receive a new goal
    while self.goal_status == actionlib.GoalStatus.ACTIVE:
      # Check if a stop navigation intent was received
      if self.stop_navigation_pending_to_attend:
        rospy.loginfo('vva_navigation_intent: Aborting "navigate to" intent because "Stop navigation" intent was received.')
        self.stop_navigation_pending_to_attend = False
        return
      rospy.sleep(0.5)
      
    time_now = rospy.Time.now()
    
    # Construct the PoseStamped with the Goal coordinates
    pose = PoseStamped()
    pose.header.stamp = time_now
    pose.header.frame_id = goal_frame

    pose.pose.position.x = goal_x
    pose.pose.position.y = goal_y
    pose.pose.position.z = 0.0
    
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0
    
    # Check if the coordinates need to be transformed to the map frame
    if goal_frame != self.map_frame:
      try:
        self.tf_listener.waitForTransform(self.map_frame, goal_frame, time_now, rospy.Duration(4.0))
        pose = self.tf_listener.transformPose(self.map_frame, pose)
        rospy.loginfo('vva_navigation_intent: Goal coordinates transformed: frame A: %s (%.2f, %.2f), frame B: %s (%.2f, %.2f)',
                      goal_frame, goal_x, goal_y, pose.header.frame_id, pose.pose.position.x, pose.pose.position.y)
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, TransformException) as e:
        rospy.logwarn("vva_navigation_intent: Frame transform (TF) exception: " + str(e))

    # Send the goal
    
    # Check if a stop navigation intent was received
    if self.stop_navigation_pending_to_attend:
      rospy.loginfo('vva_navigation_intent: Aborting "navigate to" intent because "Stop navigation" intent was received.')
      self.stop_navigation_pending_to_attend = False
      return
        
    rospy.loginfo('vva_navigation_intent: Sending goal: (x=%.1f, y=%.1f) in "%s" frame.',
                pose.pose.position.x, pose.pose.position.y, pose.header.frame_id)

    self.simple_goal_pub.publish(pose)
    
    # Wait for the result
    rospy.loginfo('vva_navigation_intent: Waiting for the result of the goal...')
    rospy.sleep(2)
    while self.goal_status == actionlib.GoalStatus.ACTIVE:
      
      # Check if a stop navigation intent was received
      if self.stop_navigation_pending_to_attend:
        # Send a stop navigation (cancel goal) service request to vva_navigation_correction
        try:
          rospy.loginfo('vva_navigation_intent: Cancelling goal...')
          srv_stop_navigation = rospy.ServiceProxy(self.stop_navigation_service_name, Empty)
          srv_stop_navigation()
        except rospy.ServiceException as e:
          rospy.logwarn("vva_navigation_intent: Service failed: " + str(e))
          
        rospy.loginfo('vva_navigation_intent: Aborting "navigate to" intent because "Stop navigation" intent was received.')
        self.stop_navigation_pending_to_attend = False
        # ~ return
        
      rospy.sleep(0.5)
      
    goal_elapsed_time = (rospy.Time.now() - pose.header.stamp).to_sec()

    if self.goal_status == actionlib.GoalStatus.SUCCEEDED:
      rospy.loginfo('vva_navigation_intent: Goal reached. Elapsed time: %.1f seconds', goal_elapsed_time)
    elif self.goal_status == actionlib.GoalStatus.ABORTED: 
      rospy.loginfo('vva_navigation_intent: Goal failed. Elapsed time: %.1f seconds', goal_elapsed_time)
    else:
      rospy.loginfo('vva_navigation_intent: Goal status code: %d. Elapsed time: %.1f seconds', self.goal_status, goal_elapsed_time)
    
    return self.goal_status, goal_elapsed_time
    
  
  # ==================================================
  # Resolve location
  # ==================================================
  def resolve_location(self, location_text):
    
    exists = False
    x = 0.0
    y = 0.0
    frame = ""
    
    ## DEBUG
    # ~ rospy.loginfo('vva_navigation_intent: Resolving location "%s"', location_text)
    
    for location in self.location_names:
      ## DEBUG
      # ~ rospy.loginfo('vva_navigation_intent: Looking for "%s"', location['name'])
      if location_text.find(location['name']) != -1:
        exists = True
        x = location['x']
        y = location['y']
        frame = location['frame']
        
        rospy.loginfo('vva_navigation_intent: Location found "%s"', location['name'])

        break
    
    return exists, x, y, frame
  
  
  
  # ==================================================
  # Start patrolling
  # ==================================================
  def start_patrolling(self):
    
    goal_status_list = [0] * len(self.patrolling_itinerary)
    goal_elapsed_times_list = [0.0] * len(self.patrolling_itinerary)
    
    for i, itin_location in enumerate(self.patrolling_itinerary):
      goal_status_list[i], goal_elapsed_times_list[i] = self.navigate_to(itin_location)
      
      # If goal was cancelled by the user, don't continue with the patrolling
      if goal_status_list[i] == actionlib.GoalStatus.PREEMPTED or goal_status_list[i] == actionlib.GoalStatus.RECALLED:
        break
      
      
    # Once all the goals (locations) are processed, print a status summary
    rospy.loginfo('vva_navigation_intent: --------------------------------------------')
    rospy.loginfo('vva_navigation_intent: Patrolling Goals status:')
    rospy.loginfo('vva_navigation_intent: location_name,status,time_elapsed(s)')
    i = 0
    while i < len(self.patrolling_itinerary):
      rospy.loginfo("vva_navigation_intent: %s,%d,%.1f", self.patrolling_itinerary[i], goal_status_list[i], goal_elapsed_times_list[i])
      i += 1
    rospy.loginfo('vva_navigation_intent: --------------------------------------------')




  # ==================================================
  # Update function
  # ==================================================
  def update(self):
    
    if self.navigate_to_pending_to_attend:
      rospy.loginfo('vva_navigation_intent: Received intent execution request: "navigate_to: ' + self.navigate_to_arg + '"')
      self.navigate_to(self.navigate_to_arg)
      self.navigate_to_pending_to_attend = False

    elif self.start_patrolling_pending_to_attend:
      rospy.loginfo('vva_navigation_intent: Received intent execution request: "start_patrolling"')
      self.start_patrolling()
      self.start_patrolling_pending_to_attend = False
    


  # ==================================================
  # Main loop
  # ==================================================
  def spin(self):
    rospy.loginfo("vva_navigation_intent: Start")
    rate = rospy.Rate(self.rate)
    rospy.on_shutdown(self.shutdown)
    
    while not rospy.is_shutdown():
      self.update()
      rate.sleep()
    rospy.spin()


  # ==================================================
  # If shutdown
  # ==================================================

  def shutdown(self):
    rospy.loginfo("vva_navigation_intent: Stop. CHAO PESCAO!")
    # Stop message
    rospy.sleep(1)
    

# ==================================================
# Main
# ==================================================
def main():
  navigation_intent = NavigationIntent()
  navigation_intent.spin()

if __name__ == '__main__':
  main()







