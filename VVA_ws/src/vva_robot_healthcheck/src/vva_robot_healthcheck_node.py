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
# andres.arboleda AT gmail DOT com, (nov/2020)
#------------------------------------------------------------------------------


import rospy
from std_msgs.msg import UInt16, Float64
from vva_msgs.msg import NavCorrectionStatus
from std_srvs.srv import Empty
import actionlib


# Receives services and topics with reports of the health of the system and
# provides feedback to the user using the Kinect's LED or any other available
# means
class RobotHealthcheck:
  def __init__(self):
    rospy.init_node('vva_robot_healthcheck')
    
    # Subscribed topics
    self.goal_status_sub = rospy.Subscriber('vva_navigation_correction/status', NavCorrectionStatus, self.goal_status_topic_callback)
    
    # Published topics
    self.kinect_led_option_pub = rospy.Publisher('vva_kinect/led_option', UInt16, queue_size=10)
    self.kinect_tilt_angle_pub = rospy.Publisher('vva_kinect/tilt_angle', Float64, queue_size=10)
    
    # Published services
    self.report_wifi_ok_srv    = rospy.Service('~report_wifi_ok',    Empty, self.report_wifi_ok_srv_callback)
    self.report_wifi_error_srv = rospy.Service('~report_wifi_error', Empty, self.report_wifi_error_srv_callback)

    # Parameters
    self.rate                  = rospy.get_param('~rate', 10)
    
    # Global variables for subscribed topics
    self.goal_status = actionlib.GoalStatus.ACTIVE
    

  # ==================================================
  # Topics and services callback functions
  # ==================================================
  def goal_status_topic_callback(self,msg):
    self.goal_status = msg.status

  def report_wifi_ok_srv_callback(self,req):
    rospy.loginfo("vva_robot_healthcheck: WiFi ok reported.")
    
    # Publish topics asking the Kinect to put the LED in blinking green
    kinect_led_val = UInt16()
    kinect_led_val.data = 4      # blinking green
    self.kinect_led_option_pub.publish(kinect_led_val)
    
    # If the robot is not in the middle of a navigation goal processing
    if self.goal_status != actionlib.GoalStatus.ACTIVE:
      rospy.loginfo("vva_robot_healthcheck: No nav. goal active, asking the Kinect to set the tilt to 0 degrees.")
      # Set the tilt in 0 degrees
      kinect_tilt_val = Float64()
      kinect_tilt_val.data = 0
      self.kinect_tilt_angle_pub.publish(kinect_tilt_val)
    return []
    
  def report_wifi_error_srv_callback(self,req):
    rospy.loginfo("vva_robot_healthcheck: WiFi error reported")
    
    # Publish topics asking the Kinect to put the LED in blinking red/orange
    kinect_led_val = UInt16()
    kinect_led_val.data = 6      # blinking red/orange
    self.kinect_led_option_pub.publish(kinect_led_val)
    
    # If the robot is not in the middle of a navigation goal processing
    if self.goal_status != actionlib.GoalStatus.ACTIVE:
      rospy.loginfo("vva_robot_healthcheck: No nav. goal active, asking the Kinect to set the tilt to +20 degrees.")
      # Set the tilt in +20 degrees
      kinect_tilt_val = Float64()
      kinect_tilt_val.data = 20
      self.kinect_tilt_angle_pub.publish(kinect_tilt_val)
    return []


  # ==================================================
  # Update function
  # ==================================================
  # ~ def update(self):
    


  # ==================================================
  # Main loop
  # ==================================================
  def spin(self):
    rospy.loginfo("vva_robot_healthcheck: Start")
    rate = rospy.Rate(self.rate)
    rospy.on_shutdown(self.shutdown)
    
    while not rospy.is_shutdown():
      # ~ self.update()
      rate.sleep()
    rospy.spin()


  # ==================================================
  # If shutdown
  # ==================================================

  def shutdown(self):
    rospy.loginfo("vva_robot_healthcheck: Stop. CHAO PESCAO!")
    # Stop message
    rospy.sleep(1)
    

# ==================================================
# Main
# ==================================================
def main():
  robot_healthcheck = RobotHealthcheck()
  robot_healthcheck.spin()

if __name__ == '__main__':
  main()










