#!/usr/bin/env python

#------------------------------------------------------------------------------
# Copyright (c) 2021, Andres A. <andres.arboleda AT gmail DOT com>
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
# andres.arboleda AT gmail DOT com, (mar/2021)
#------------------------------------------------------------------------------


import rospy
from std_msgs.msg import UInt16
from std_msgs.msg import UInt8
from std_srvs.srv import Empty
import os
import roslaunch

# status topic constants
IDLE     = 0
MAPPING  = 1

# In charge of managing the ROS system based on communication with the FrontEnd.
# Launches and closes rtabmap launchfiles.
# Publishes the size of the map file that is being created.
class WebRobotManagement:
  def __init__(self):
    rospy.init_node('vva_robot_management')

    # Subscribed topics
    # self.goal_status_sub = rospy.Subscriber('vva_navigation_correction/status', NavCorrectionStatus, self.goal_status_topic_callback)

    # Published topics
    self.status_pub   = rospy.Publisher('~status',    UInt8, queue_size=10)
    self.map_size_pub = rospy.Publisher('~map_size', UInt16, queue_size=10)

    # Published services
    self.stop_mapping_srv    = rospy.Service('~stop_mapping',  Empty, self.stop_mapping_srv_callback)
    self.start_mapping_srv   = rospy.Service('~start_mapping', Empty, self.start_mapping_srv_callback)

    # Parameters
    self.rate               = rospy.get_param('~rate', 1)
    self.rtabmap_launchfile = rospy.get_param('~rtabmap_launchfile',
      "/home/theuser/AAData/Documents2/ROS/VehiculoVigilanciaAutonomo/VVA_ws/src/vva_navigation/launch/vva_rtabmap_simulation.launch")

    # Global variables for subscribed topics
    # self.goal_status = actionlib.GoalStatus.ACTIVE

    # State
    self.isMappingStarted      = False
    self.pendingToStartMapping = False
    self.pendingToStopMapping  = False

    # Other global vars
    self.map_db_file           = ""
    self.rtabmap_launch_parent = None



  # ==================================================
  # Topics and services callback functions
  # ==================================================
  # def goal_status_topic_callback(self,msg):
  #   self.goal_status = msg.status

  def start_mapping_srv_callback(self,req):
    rospy.loginfo("vva_robot_management: start mapping service called.")
    self.pendingToStartMapping = True
    return []

  def stop_mapping_srv_callback(self,req):
    rospy.loginfo("vva_robot_management: stop mapping service called.")
    self.pendingToStopMapping = True
    return []



  # ==================================================
  # Update function
  # ==================================================
  def update(self):

    # Execute only once, when the start_mapping service is called
    if self.pendingToStartMapping:
      # Start the rtabmap mapping
      cli_args = [self.rtabmap_launchfile, 'localization:=false']
      roslaunch_args = cli_args[1:]
      roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
      uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
      self.rtabmap_launch_parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
      self.rtabmap_launch_parent.start()

      # Update the state
      self.isMappingStarted = True
      self.pendingToStartMapping = False

      rospy.loginfo("vva_robot_management: mapping (rtabmap) started.")

      # Get the database_path parameter from rtabmap
      self.map_db_file = rospy.get_param('/rtabmap_jnano/rtabmap/database_path', "")
      rospy.loginfo("vva_robot_management: The rtabmap/database_path is: %s", self.map_db_file)

    # Execute several times while rtabmap is running
    if self.isMappingStarted:
      # Check the map database file size
      try :
        file_size_MB = int( os.path.getsize(self.map_db_file)/(1024*1024) )
        #Publish the topic with the map size
        self.map_size_pub.publish(file_size_MB)
      except Exception as err:
        rospy.loginfo("vva_robot_management: An exception happend: " + str(err))

      self.status_pub.publish(MAPPING)
    else:
      self.status_pub.publish(IDLE)

    # Execute only once, when the stop_mapping service is called
    if self.pendingToStopMapping:
      # Stop the rtabmap mapping
      self.rtabmap_launch_parent.shutdown()

      # Update the state
      self.isMappingStarted = False
      self.pendingToStopMapping = False

      rospy.loginfo("vva_robot_management: mapping (rtabmap) stopped.")



  # ==================================================
  # Main loop
  # ==================================================
  def spin(self):
    rospy.loginfo("vva_robot_management: Start")
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
    rospy.loginfo("vva_robot_management: Stop. CHAO PESCAO!")
    # Stop message
    rospy.sleep(1)


# ==================================================
# Main
# ==================================================
def main():
  robot_management = WebRobotManagement()
  robot_management.spin()

if __name__ == '__main__':
  main()
