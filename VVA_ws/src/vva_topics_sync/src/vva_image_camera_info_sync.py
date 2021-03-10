#!/usr/bin/env python

#------------------------------------------------------------------------------
# Copyright (c) 2020, Andres A. <andres.arboleda AT gmail DOT com>
# All rights reserved.
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
#------------------------------------------------------------------------------

import rospy
import message_filters
from sensor_msgs.msg import Image, CameraInfo

# Synchronizes the image and camera info topics

# ==================================================
# Read the subscribed topics
# ==================================================
def callback(image, camera_info):
  output_image_topic_pub.publish(image)
  output_camera_info_topic_pub.publish(camera_info)


# ==================================================
# Main
# ==================================================
rospy.init_node('vva_image_camera_info_sync')

# Input topics
input_image_topic = rospy.get_param('~input_image_topic','/camera/depth_registered/image_raw')
input_camera_info_topic = rospy.get_param('~input_camera_info_topic','/camera/depth_registered/camera_info')
# Output topics
output_image_topic = rospy.get_param('~output_image_topic','/synchronized/camera/depth_registered/image_raw')
output_camera_info_topic = rospy.get_param('~output_camera_info_topic','/synchronized/camera/depth_registered/camera_info')

image_sub = message_filters.Subscriber(input_image_topic, Image)
info_sub = message_filters.Subscriber(input_camera_info_topic, CameraInfo)

output_image_topic_pub = rospy.Publisher(output_image_topic, Image, queue_size=10)
output_camera_info_topic_pub = rospy.Publisher(output_camera_info_topic, CameraInfo, queue_size=10)

ts = message_filters.TimeSynchronizer([image_sub, info_sub], queue_size=10)
ts.registerCallback(callback)
rospy.loginfo("Start vva_image_camera_info_sync")
rospy.loginfo("Subscribed to:")
rospy.loginfo("   %s", input_image_topic)
rospy.loginfo("   %s", input_camera_info_topic)
rospy.spin()


