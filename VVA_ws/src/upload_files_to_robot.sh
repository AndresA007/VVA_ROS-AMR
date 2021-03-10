#/bin/bash

JNano_IP=192.168.43.167

# Upload VVA ROS packages to J-Nano:
#======================================================================================================
#~ JN_PACKAGE1=$ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_kinect_aux
#~ JN_PACKAGE2=$ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_rplidar_ros
#~ JN_PACKAGE3=$ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_jnano_consolidated
#~ JN_PACKAGE4=$ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_description
#~ JN_PACKAGE5=$ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_lidar_odom
JN_PACKAGE6=$ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_navigation
#~ JN_PACKAGE7=$ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_cliff_detector
#~ JN_PACKAGE8=$ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_cliff_detector_layer
#~ JN_PACKAGE9=$ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_msgs
#~ JN_PACKAGE10=$ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_voice_interact_server
#~ JN_PACKAGE11=$ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_user_intents
#~ JN_PACKAGE12=$ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_voice_interact_client
#~ JN_PACKAGE13=$ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_nav_test
#~ JN_PACKAGE14=$ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_base_controller
#~ JN_PACKAGE15=$ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_robot_healthcheck

scp -r \
  $JN_PACKAGE1 \
  $JN_PACKAGE2 \
  $JN_PACKAGE3 \
  $JN_PACKAGE4 \
  $JN_PACKAGE5 \
  $JN_PACKAGE6 \
  $JN_PACKAGE7 \
  $JN_PACKAGE8 \
  $JN_PACKAGE9 \
  $JN_PACKAGE10 \
  $JN_PACKAGE11 \
  $JN_PACKAGE12 \
  $JN_PACKAGE13 \
  $JN_PACKAGE14 \
  $JN_PACKAGE15 \
  ubuntu@$JNano_IP:/home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/


# Upload the scripts to run the nodes on backgroud:
#--------------------------------------------
#~ scp -r \
  #~ $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/RunAsBackgroundScript \
  #~ ubuntu@$JNano_IP:/home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6.1/


# Upload the Email Notification And Watchdog Scripts:
#--------------------------------------------
#~ scp -r \
  #~ $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/EmailNotificationAndWatchdogScripts/sendEmail.py \
  #~ $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/EmailNotificationAndWatchdogScripts/WiFiWatchdog.sh \
  #~ $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/EmailNotificationAndWatchdogScripts/sendMyIPByEmail.sh \
  #~ ubuntu@$JNano_IP:/home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6.1/EmailNotificationAndWatchdogScripts/


# Upload the DeepSpeech module to the J-Nano:
#--------------------------------------------
#~ scp -r \
  #~ $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/DeepSpeechModule/src \
  #~ $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/DeepSpeechModule/run_DS_module_jnano.sh \
  #~ ubuntu@$JNano_IP:/home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6.1/DeepSpeechModule/



