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
from geometry_msgs.msg import PoseWithCovarianceStamped
from vva_msgs.msg import NavCorrectionStatus
import actionlib


class VVATopicAnalyzer:
  def __init__(self):
    rospy.init_node('vva_topic_analyzer')
    self.nav_corr_status_sub = rospy.Subscriber('vva_navigation_correction/status', NavCorrectionStatus, self.nav_corr_status_callback)
    self.rtabmap_localization_sub = rospy.Subscriber('/rtabmap_laptop/localization_pose', PoseWithCovarianceStamped, self.rtabmap_localization_callback)
    
    # Parameters
    self.rate                  = rospy.get_param('~rate', 0.2)
    self.time_after_nav_active = rospy.get_param('~time_after_nav_active', 60)
    
    # Topics global variables
    self.nav_corr_status = actionlib.GoalStatus.PENDING
    self.loop_closure_count = 0
    
    # Statistics file
    self.vel_stats_file = None
    
    # Other global variables
    self.was_active = False
    self.start_time = rospy.Time(0)
    
    
  # ==================================================
  # Read the subscribed topics
  # ==================================================
  def nav_corr_status_callback(self,msg):
    self.nav_corr_status = msg.status
    
  def rtabmap_localization_callback(self,msg):
    self.loop_closure_count += 1


  # ==================================================
  # update function
  # ==================================================
  def update(self):
    
    if self.nav_corr_status == actionlib.GoalStatus.ACTIVE:
      # Count loop closures
      rospy.loginfo("nav_corr_status=ACTIVE: %f,%d\n", rospy.Time.now().to_sec(), self.loop_closure_count)
      self.vel_stats_file.write("%f,%d,Active\n" % (
                                 rospy.Time.now().to_sec(),
                                 self.loop_closure_count
                                 ) )
      self.was_active = True
      
    if self.nav_corr_status != actionlib.GoalStatus.ACTIVE and self.was_active:
      # Start counting the time
      self.start_time = rospy.Time.now()
      self.was_active = False

    if (rospy.Time.now() - self.start_time).to_sec() < self.time_after_nav_active:
      # Count loop closures
      rospy.loginfo("In time period: %f,%d\n", rospy.Time.now().to_sec(), self.loop_closure_count)
      self.vel_stats_file.write("%f,%d,Waiting\n" % (
                                 rospy.Time.now().to_sec(),
                                 self.loop_closure_count
                                 ) )
      

  # ==================================================
  # Main loop
  # ==================================================
  def spin(self):
    rospy.loginfo("Start vva_topic_analyzer")
    rate = rospy.Rate(self.rate)
    
    rospy.on_shutdown(self.shutdown)
    
    # Write headers in statistics file
    self.vel_stats_file =  open("/home/theuser/AAData/Documents2/ROS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_nav_test/topic_analyzer_stats/loop_closure_stats.csv", 'w')
    self.vel_stats_file.write(
                  "Timestamp," +
                  "Loop_closures," +
                  "Navigation_status"
                  "\n"
                  )

    while not rospy.is_shutdown():
      self.update()
      rate.sleep()
    rospy.spin();



  # ==================================================
  # If shutdown
  # ==================================================
  def shutdown(self):

    self.vel_stats_file.close()

    rospy.loginfo("Stop vva_topic_analyzer. CHAO PESCAO!")
    # Stop message
    rospy.sleep(1)    

  
    
# ==================================================
# Main
# ==================================================
def main():
  vva_topic_analyzer = VVATopicAnalyzer();
  vva_topic_analyzer.spin()


if __name__ == '__main__':
  main(); 

