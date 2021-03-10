#!/usr/bin/env python

# Send velocity commands to test the movement of the robot

import rospy
from time import sleep

from geometry_msgs.msg import Twist

rospy.init_node('test_nav')
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
move_cmd = Twist()

move_cmd.linear.x = 0.0
move_cmd.angular.z = 0.0
rospy.loginfo("Enviando topic v: %.2f, w: %.2f", move_cmd.linear.x, move_cmd.angular.z)
cmd_vel_pub.publish(move_cmd)
sleep(7.0)

move_cmd.linear.x = 10.0
move_cmd.angular.z = 0.0
rospy.loginfo("Enviando topic v: %.2f, w: %.2f", move_cmd.linear.x, move_cmd.angular.z)
cmd_vel_pub.publish(move_cmd)
sleep(3.5)

move_cmd.linear.x = 0.0
move_cmd.angular.z = 0.0
rospy.loginfo("Enviando topic v: %.2f, w: %.2f", move_cmd.linear.x, move_cmd.angular.z)
cmd_vel_pub.publish(move_cmd)
sleep(0.5)



# ~ for i in range(0, 1):
  # ~ rospy.loginfo("Ciclo %d",i+1)
  # ~ move_cmd.linear.x = 0.2
  # ~ move_cmd.angular.z = 0.0
  # ~ rospy.loginfo("Enviando topic v: %.2f, w: %.2f", move_cmd.linear.x, move_cmd.angular.z)
  # ~ cmd_vel_pub.publish(move_cmd)
  # ~ sleep(0.5)
  
  # ~ move_cmd.linear.x = 0.0
  # ~ move_cmd.angular.z = 0.0
  # ~ rospy.loginfo("Enviando topic v: %.2f, w: %.2f", move_cmd.linear.x, move_cmd.angular.z)
  # ~ cmd_vel_pub.publish(move_cmd)
  # ~ sleep(1)
  
  # ~ move_cmd.linear.x = 0.1
  # ~ move_cmd.angular.z = 0.0
  # ~ rospy.loginfo("Enviando topic v: %.2f, w: %.2f", move_cmd.linear.x, move_cmd.angular.z)
  # ~ cmd_vel_pub.publish(move_cmd)
  # ~ sleep(2)
  
  # ~ move_cmd.linear.x = 0.0
  # ~ move_cmd.angular.z = 0.0
  # ~ rospy.loginfo("Enviando topic v: %.2f, w: %.2f", move_cmd.linear.x, move_cmd.angular.z)
  # ~ cmd_vel_pub.publish(move_cmd)
  # ~ sleep(1)







