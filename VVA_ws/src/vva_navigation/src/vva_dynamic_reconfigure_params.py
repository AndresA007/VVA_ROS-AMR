#!/usr/bin/env python

import rospy
#import move_base_msgs.msg

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import actionlib
import move_base_msgs.msg
from std_srvs.srv import Empty
from vva_msgs.msg import OdomCorrectionStatus
from vva_msgs.msg import NavCorrectionStatus

import time

import dynamic_reconfigure.client   #
from sensor_msgs.msg import LaserScan ##

msgLidarScan = 'valor'

class VVADynamicReconfigure:

  def __init__(self):

    rospy.init_node('vva_dynamic_reconfigure')

    # Subscribed topics
    self.lidarScan_sub =  rospy.Subscriber('/rplidar_scan',  LaserScan, self.laserscan_callback)
    

    # Parameters
    self.rate = rospy.get_param('~rate', 0.2) #0.2

    # Global variables for subscribed topics
    self.lidarScan_value = None

# ==================================================
  # Topics and services callback functions
  # ==================================================
  def laserscan_callback(self,msg):
     self.lidarScan_value = msg

# ==================================================
  # update function
  # ==================================================

  def dynamic_reconfigure_params(self):

    lsv = self.lidarScan_value

    if lsv != None:

      vLS = self.lidarScan_value.ranges
      rospy.loginfo(vLS)
      rospy.loginfo("Tamano: ")
      tamV = len(self.lidarScan_value.ranges) #Promedio, con el promedio comenzar a probar para acomodar la inflation
      rospy.loginfo(tamV)

      #max 
      maxV = self.lidarScan_value.range_max

      inf = float('inf')

      valor = vLS.count(inf)
      rospy.loginfo("# VECES QUE SE REPITE inf: ")
      rospy.loginfo(valor)

      obsm = 200
      cont = 0
      vN = [0]*tamV

      while cont < tamV:

        if vLS[cont] == inf:
            vN[cont] = maxV
        else:
            vN.append(vLS[cont])

            #Obstaculo mas cercano - menor valor 
            if vLS[cont] != 0:
                if vLS[cont] < obsm:
                    obsm = vLS[cont]
                    
        cont += 1
      

      rospy.loginfo("------- OBSTACULO MAS PEQUENO:")
      rospy.loginfo(obsm)

      cont = 0 
      cont2 = 0

      tvN = len(vN)

      #Se usa porque genera un vector de 572, llenando en cero el resto de campos diferentes de los 300
      while cont < tvN:

          if vN[cont] != 0:

            cont2 += 1

          cont += 1

      promedio = sum(vN)/cont2
      rospy.loginfo("Promedio: ")
      rospy.loginfo(promedio)
      rospy.loginfo("Tamano: ")
      rospy.loginfo(len(vN))
      rospy.loginfo(cont2)

      if promedio < 2.11 or obsm < 0.5:   #obsm < 0.65

        client= dynamic_reconfigure.client.Client('/move_base/local_costmap/local_inflation_layer')
        params = {'inflation_radius' : 0.2}
        config = client.update_configuration(params)
        rospy.loginfo("************************************************************************")
      
        client= dynamic_reconfigure.client.Client('/move_base/TrajectoryPlannerROS')
        params = {'max_vel_x' : 0.1}
        config = client.update_configuration(params)
        rospy.loginfo("************************************************************************")
      
      else:
        client= dynamic_reconfigure.client.Client('/move_base/local_costmap/local_inflation_layer')
        params = {'inflation_radius' : 0.55}
        config = client.update_configuration(params)
        rospy.loginfo("************************************************************************")
 
        client= dynamic_reconfigure.client.Client('/move_base/TrajectoryPlannerROS')
        params = {'max_vel_x' : 0.8} #0.93
        config = client.update_configuration(params)
        rospy.loginfo("************************************************************************")
      

    time.sleep(5) 

    return []



# ==================================================
  # Main loop
  # ==================================================
  def spin(self):
    rospy.loginfo("Start vva_dynamic_reconfigure_params")
    rate = rospy.Rate(self.rate)
    
    rospy.on_shutdown(self.shutdown)

    while not rospy.is_shutdown():
      self.dynamic_reconfigure_params()
      rate.sleep()
      rospy.loginfo(self.rate)
    rospy.spin()



  # ==================================================
  # If shutdown
  # ==================================================
  def shutdown(self):

    rospy.loginfo("Stop vva_dynamic_reconfigure_params. CHAO PESCAO!")
    # Stop message
    rospy.sleep(1)    

  
    
# ==================================================
# Main
# ==================================================
def main():
   vva_dynamic_reconfigure = VVADynamicReconfigure();
   vva_dynamic_reconfigure.spin()


if __name__ == '__main__':
  main(); 

