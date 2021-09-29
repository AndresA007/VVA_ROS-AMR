#!/usr/bin/python
import rospy
import roslib
import math 
import numpy
import tf

# Messages
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist


class OdometryWheelsEncoders:
    def __init__(self):
      rospy.init_node('vva_base_odometry_wheels_encoders')

      self.vva_angular_vel_left_front_wheel_enc_sub = rospy.Subscriber('vva_angular_vel_left_front_wheel_enc', Float32, self.angular_vel_left_front_wheel_enc_callback)
      self.vva_angular_vel_left_back_wheel_enc_sub = rospy.Subscriber('vva_angular_vel_left_back_wheel_enc', Float32, self.angular_vel_left_back_wheel_enc_callback)
      self.vva_angular_vel_right_front_wheel_enc_sub = rospy.Subscriber('vva_angular_vel_right_front_wheel_enc', Float32, self.angular_vel_right_front_wheel_enc_callback)
      self.vva_angular_vel_right_back_wheel_enc_sub = rospy.Subscriber('vva_angular_vel_right_back_wheel_enc', Float32, self.angular_vel_right_back_wheel_enc_callback)
                  
      
      # Angular speed of the wheels from encoders readings
      self.angular_vel_left_front_wheel_enc = 0
      self.angular_vel_left_back_wheel_enc = 0
      self.angular_vel_right_front_wheel_enc = 0
      self.angular_vel_right_back_wheel_enc = 0


    def angular_vel_left_front_wheel_enc_callback(self, msg):
      self.angular_vel_left_front_wheel_enc  = msg.data

    def angular_vel_left_back_wheel_enc_callback(self, msg):
      self.angular_vel_left_back_wheel_enc = msg.data

    def angular_vel_right_front_wheel_enc_callback(self, msg):
      self.angular_vel_right_front_wheel_enc  = msg.data

    def angular_vel_right_back_wheel_enc_callback(self, msg):
      self.angular_vel_right_back_wheel_enc = msg.data

    # ==================================================
    # Main loop
    # ==================================================
    def spin(self):

        rospy.loginfo("Start vva_odometry_wheels_encoders")
    
        rospy.on_shutdown(self.shutdown)

        while not rospy.is_shutdown():
            self.update()
          rate.sleep()
        rospy.spin();

    def update(self):




    def pose_update(self):
        lwheel_tangent_vel_enc = self.angularvel_2_tangentvel(self.lwheel_angular_vel_enc)
        rwheel_tangent_vel_enc = self.angularvel_2_tangentvel(self.rwheel_angular_vel_enc)
        self.lwheel_tangent_vel_enc_pub.publish(lwheel_tangent_vel_enc)
        self.rwheel_tangent_vel_enc_pub.publish(rwheel_tangent_vel_enc)

        pose_next = self.pose_next(lwheel_tangent_vel_enc, rwheel_tangent_vel_enc)

        cmd_vel_enc = Twist()
        cmd_vel_enc.linear.x = pose_next['v']
        cmd_vel_enc.angular.z = pose_next['w']
        self.cmd_vel_enc_pub.publish(cmd_vel_enc)

        return pose_next

    def pub_odometry(self,pose):
        # Construct odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.time_prev_update
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id
        odom_msg.pose.pose.position = Point(pose['x'], pose['y'], 0)
        odom_msg.pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0,0,pose['th']))
        #P = numpy.mat(numpy.diag([0.0]*3)) # Dummy covariance
        #odom_msg.pose.covariance = tuple(P.ravel().tolist())
        self.odom_pub.publish(odom_msg)



    def shutdown(self):
        rospy.loginfo("vva_base_controller: Stop. CHAO PESCAO!")
        rospy.sleep(1)    

    
    # ==================================================
    # Main
    # ==================================================
    def main():
        vva_odometry_wheels = OdometryWheelsEncoders();
        vva_odometry_wheels.spin()


    if __name__ == '__main__':
        main(); 

