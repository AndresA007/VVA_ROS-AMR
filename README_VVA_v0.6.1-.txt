Release Notes:
============================================================================================================================================
VehiculoVigilanciaAutonomo_v0.6:
     - Developed in the Sprint 5.
     - Supports the use of the microphones array of the Kinect by detecting a wake-word after which the user can record a 4 seconds audio clip
       with a voice command.
     - Introduces a Mobile Client App that supports the use of the microphone of the mobile phone to record voice commands.
     - The voice commands are processed by the Speech Recognition node in the package vva_voice_interact_laptop. This node is based on
       Mozilla DeepSpeech v0.6.1 and currently supports only american english language.
     - All the audio clips captured throught the microphones array of the Kinect are saved and preserved for future use as datasets for
       training DeepSpeech. The audio clips captured through the Mobile App are not preserved.
     - Introduces an architecture to implement the Intent Recognition, to interpret the Voice Commands. Several Intent Recognition nodes can be
       deployed in the package vva_user_intents, each node specialized in different tasks. Currently, there is only one node in charge of the
       navigation related tasks.
     - The Intent Recognition node in charge of the navigation tasks (vva_navigation_intent.py) supports the following commands: "navigate to",
       "start patrolling" and "stop navigation".
     - The known issue of v0.5 regading vva_navigation_correction not working on simulation is now fixed.
     - The launch files "vva_odom_correction.launch", "vva_move_base.launch" and "vva_navigation_correction.launch" were consolidated in
       one launch file called "vva_consolidated_nav.launch".
     
     - Pre-requisites:
          - Use the last version of rtabmap, which is downloaded from github and compile. The version that comes in the Ubuntu Bionic repos doesn't
            have the module "rgbd_relay". This is found in: "$ROS_HOME_WS/ROSCompiledPackages/ROSCompiledPackages_ws/src/rtabmap_ros/"
          - Use the package depth_nav_tools/laserscan_kinect, it is not available in Melodic but it can be downloaded and compiled. It is found in:
            "$ROS_HOME_WS/ROSCompiledPackages/ROSCompiledPackages_ws/src/depth_nav_tools/laserscan_kinect/"
          - If you want to make a modification of the request or the response of the ROS Service used in the Mobile App, then rosjava is needed to
            generate the .jar file. It is found in: "$ROS_HOME_WS/ROSCompiledPackages/rosjava_minimal"

	   - Included folders and files:
          20200225_CostosVehiculoVigilanciaAutonomo.ods - Summary of the costs of hardware used so far to build the vehicle
          20200504_MapeoVelocidadVehiculo_ComandosMotor.ods - Calculations about the ticks of the wheels encoders and about the motor limits to turn
                                                              in curves (not in place rotations)
          20200524_Optimization_CliffDetector_LaserscanKinect.ods - Results of optimization tests of the parameters of the nodes vva_cliff_detector and
                                                                    laserscan_kinect.
          20200620_CalculosSeleccionNuevosMotoresYDiseñoHW.ods - Estimations of the required specifications for the new hardware: motors, batteries, etc
                                                                 proposals of different designs for the new hardware and estimations of the costs.
          20200725_TestsOfHARKAudioSourceLocation.ods - Tests of HARK library about the accuracy to identify the location of the source of the sound.
          20200729_TestsOfSpeechRecognitionModels.ods - Tests about the accuracy of Speech Recognition with DeepSpeech v0.6.1 (English) and
                                                        PocketSphinx 5 pre-alpha (Spanish).
          Android - Contains the Android Studio project (VVA_Mobile_Client) of the Mobile App and a rosjava workspace (vva_rosjava_ws) to generate
                    the necessary .jar files to allow the Mobile App to communicate with the ROS service.
          ConfiguracionOpenVPN - Configuration files of OpenVPN for client and server
          Design - Design document and diagrams of VVA
          diagnosticos_tf_tree_y_rqt_graph - Images of diagrams of the TF-Tree and rqt_graph
          EstadisticasControlPIDYAcelerac - Tabulated statistics and graphs of the PID controller to define the values of the coeficients Kp, Ki y Kd.
                                            Statistics to determine the acceleration limits of the vehicle for the configuration of the base_local_planner.
                                            Statistics of the quantity of loop closures reported by rtabmap in different light conditions.
          OtherScripts - Currently, only has a script to prepare the datasets for training a DeepSpeech model.
          SMTPNotificationScript - Crontab scripts in charge of email notifications of events.
          VVA_ws/src/upload_files_to_rbpi.sh - Script to copy selected packages from the Laptop to the RbPi.
          ../Backup/create_backup.sh - Creates a backup upon its execution
     
	   - Included ROS packages:
          vva_base_controller - Control of the motors of the wheels. Runs on the RbPi.
          vva_cliff_detector - This module is a fork of the code of depth_nav_tools to slightly change the behavior regarding the topics.
                               It is in charge of detecting holes or cliffs.
          vva_cliff_detector_layer - Fork of the code of depth_nav_tools to reduce the verbosty of the module. It is in charge of including
                                     the holes and cliffs in the costmap.
          vva_description - Contains the URDF model and the launch files for rviz and for publishing the TF of the URDF model.
          vva_gazebo - Gazebo worlds.
          vva_kinect_aux - In charge of changing the tilt of the Kinect and of publishing the position of the joints according with the
                           Kinect's IMU. Runs on the RbPi.
          vva_lidar_filter - LaserScans filter used to reduce the angle covered by the laserscan_kinect and hence remove the noise generated
                             by the Kinect in a small section of the field of view.
          vva_lidar_odom - LaserScan based odometry, uses the icp_odometry module of rtabmap.
          vva_msgs - Package that contains the defintions of the customized messages and services used in VVA.
          vva_navigation - Contains the modules related to the mapping, localization and navigation stack, including rtabmap and move_base.
          vva_nav_test - Manual control of the robot based on the keyboard arrow keys and also by scripting. It also contains a node to
                         generate customized statistics in tables based on selected topics.
          vva_raspi_consolidated - Consolidated launch file that starts in the RbPi: OpenNI Kinect, kinect_aux, rgbd_sync, base encoders publisher
                                   and base controller. Besides there is a launch file to start the RP-Lidar.
          vva_rplidar_ros - (New in v0.6) Fork of the rplidar_ros package. It is modified to add options that enable the change of the values of the
                            Lidar intensities. Runs on the RbPi.
          vva_topics_sync - Package that uses message_filters to synchronize the image_raw and camera_info topics generated by the Kinect.
          vva_user_intents - (New in v0.6) This package contains nodes in charge of the execution of actions identified during
                             the Intent Recognition. Besides is responsible for the high level behaviors of the robot.
          vva_voice_interact_laptop - (New in v0.6) This package contains the Speech Recognition module, which is based on DeepSpeech v0.6.1.
                                      It also performs the mapping between transcripts and intent services.
          vva_voice_interact_rbpi - (New in v0.6) This node is in charge of interfacing with the microphones array of the Kinect and perform the
                                    wake-word detection, which is based on Porcupine. Runs on the RbPi.
     
     - Known issues:
          - The model is not shown in Gazebo, when the joints are of type "revolute". For example, this happened with the joints between the
            Kinect and the base.
          - vva_cliff_detector doesn't work when simulated in Gazebo, it shows the error: "Could not perform stairs detection:
            Depth image has unsupported encoding: 32FC1"
          - Sometimes the depth image transmission has interruptions or it just doesn't arrive any more, hence the navigation stack
            stops working. The cause of this issue is unknown so far, maybe it is due to the weakness of the WiFi signal in some locations.
          - When voice commands are captured using the microphones array of the Kinect, the Speech Recognition doesn't work accurately. Maybe
            due to the environmental noise and because the microphones are far from the person speaking.
     
     - Commands to test it in simulation:
     
          # Deploy the model in Gazebo:
             roslaunch vva_gazebo vva_world.launch
          # Start rviz and the state_publishers:
             roslaunch vva_description rviz_rtabmap.launch simulation:=true
          # -------------------------------------------------------------------------------------------------------   
          # Mapping:
          # -------------------------------------------------------------------------------------------------------   
          # Start rtabmap:
             roslaunch vva_navigation vva_rtabmap.launch simulation:=true localization:=false
          # Use the teleop:
             roslaunch vva_nav_test vva_teleop2.launch
          # Move the robot to generate the map, turn 360° in different places,
          # In the Laptop: If you want to restart the map execute:
             rosservice call /rtabmap_laptop/reset
          # -------------------------------------------------------------------------------------------------------   
          # Once done, close rtabmap and the teleop.
          # Localization:
          # -------------------------------------------------------------------------------------------------------   
          # Start rtabmap:
             roslaunch vva_navigation vva_rtabmap.launch simulation:=true localization:=true
          # Start the modules: navigation (move_base), image-camera_info sync, laserscan_kinect, laserscan_kinect_filter and vva_navigation_correction:
             roslaunch vva_navigation vva_consolidated_nav.launch simulation:=true
          # Update the location coordinates in navigation_intent_params_simulation.yaml and start the navigation-intent node:
             roslaunch vva_user_intents vva_user_intents.launch simulation:=true
          # Start the Voice Interaction Speech Recognition module:
             roslaunch vva_voice_interact_laptop vva_voice_interact_laptop.launch
     
     - Commands to test it in the hardware:
     
          # In the RbPi: If it is required, turn off the USB ports: LIDAR and Kinect (This will disable all the ports):
             # Turn off USB ports:
               sudo su -
               echo '1-1' > /sys/bus/usb/drivers/usb/unbind
             # Turn on USB ports:
               sudo su -
               echo '1-1' > /sys/bus/usb/drivers/usb/bind
            # If it is required only to stop or start the RP-Lidar motor:
               rosservice call /stop_motor
               rosservice call /start_motor
          # In the RbPi: Check that NTP synchronization with the laptop is working (ntpq -p), if it is not execute:
             sudo systemctl stop ntp
             sudo ntpdate <Dir_IP_de_la_Laptop> # Execute several times
             sudo systemctl start ntp
             ntpq -p
          # In the RbPi: If it is required, calibrate the wheels motors:
             rm /home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6/VVA_ws/src/vva_base_controller/wheels_calibration/wheels_calibration.txt
             roslaunch vva_base_controller vva_wheels_calibration.launch
          # -------------------------------------------------------------------------------------------------------
          # In the RbPi: Start the module: rplidar (it was separated from the consolidated launch file because it failed to start scanning):
             roslaunch vva_raspi_consolidated vva_raspi_rplidar.launch
          # In the RbPi: Start the modules: kinect, kinect_aux, rgbd_sync, encoders publisher y base controller:
             roslaunch vva_raspi_consolidated vva_raspi_consolidated_rtabmap.launch
          # In the RbPi: (Optional) Start the Kinect microphone Voice Interaction wake word detection module:
             roslaunch vva_voice_interact_rbpi vva_voice_interact_rbpi.launch
          # -------------------------------------------------------------------------------------------------------
          # In the Laptop: Start rviz and the state_publishers:
             roslaunch vva_description rviz_rtabmap.launch simulation:=false
          # In the Laptop: Start the odometry based on laserscan:
             roslaunch vva_lidar_odom vva_lidar_odom.launch
          # -------------------------------------------------------------------------------------------------------   
          # Mapping:
          # -------------------------------------------------------------------------------------------------------   
          # In the Laptop: Start the modules: rgbd_relay, el rtabmap y el rtabmapviz, in mapping mode:
             roslaunch vva_navigation vva_rtabmap.launch simulation:=false localization:=false
          # In the Laptop: Use the teleop:
             roslaunch vva_nav_test vva_teleop2.launch
          # Move the robot to generate the map, turn 360° in different places,
          # In the Laptop: If you want to restart the map execute:
             rosservice call /rtabmap_laptop/reset
          # -------------------------------------------------------------------------------------------------------   
          # Once done, close rtabmap and the teleop.
          # Localization:
          # -------------------------------------------------------------------------------------------------------   
          # In the Laptop: Start the modules: rgbd_relay, rtabmap and rtabmapviz, in localization mode:
             roslaunch vva_navigation vva_rtabmap.launch simulation:=false localization:=true
          # In the Laptop: Start the modules: odometry correction, navigation (move_base), image-camera_info sync,
          # vva_cliff_detector, laserscan_kinect, laserscan_kinect_filter and vva_navigation_correction:
             roslaunch vva_navigation vva_consolidated_nav.launch simulation:=false
          # In the Laptop: Update the location coordinates in navigation_intent_params.yaml and launch the navigation-intent node:
             roslaunch vva_user_intents vva_user_intents.launch simulation:=false
          # In the Laptop: Start the Voice Interaction Speech Recognition module:
             roslaunch vva_voice_interact_laptop vva_voice_interact_laptop.launch
             
============================================================================================================================================
VehiculoVigilanciaAutonomo_v0.6.1:
     - Developed in the Sprints 2 and 2.1.
     - All the ROS packages and nodes that were previously intended to run on the Raspberry Pi or the Laptop, were migrated to run on the
       Nvidia Jetson Nano (J-Nano). Except by rviz, rtabmapviz and the teleop.
     - When running on hardware, the state publishers are now started from “vva_jnano_consolidated/ vva_jnano_lowlevel_consolidated.launch”.
       when running on simulation, they are started from “vva_description/ rviz_rtabmap_simulation.launch”.
     - The version of rtabmap used is v0.20, which comes in the apt repos. Is no longer required to compile it because the "rgbd_relay" node
       is not used, because all the nodes run on the same computer (J-Nano).
     - Two new launch files were done to decouple the rtabmap (in the J-Nano) and rtabmapviz (in the Laptop): "vva_rtabmap_hw.launch" and
       "vva_rtabmapviz_hw.launch". The simulation will be managed in a separate launch file: "vva_rtabmap_simulation.launch".
     - DeepSpeech was decoupled from the package "vva_voice_interact_server" (previously called "vva_voice_interact_laptop"), now it runs as
       an independent process under Python 3.7 (ROS Melodic runs on Python 2.7). The communication between DeepSpeech and ROS is done using
       sockets and a shared file. DeepSpeech version used: v0.8.2.
     - The package "vva_voice_interact_server" now notifies the user through the Kinect's LED when a voice command is received and if it was
       recognized as a valid command or not.
     - The URDF model was updated to match the new vehicle, called VAA (Vehiculo de Asistencia Autonomo), based on the vehicle design number 6.
       CAD file: "20201001_NuevoHW_diseno6_v2.1.2_temp.FCStd".
     - A new teleop node is available. This can move the vehicle in curves combining translation and rotation.
       To invoke use: "roslaunch vva_nav_test vva_teleop3.launch".
     - An Arduino Mega2560 is now used to manage all the low level tasks, and to avoid the usage of the GPIO ports of the J-Nano, to prevent
       damages in the J-Nano due to over-voltages.
     - The package "vva_base_controller" was modified to split the functionality with the Arduino. The communication with the Arduino is
       implemented using "rosserial_arduino".
     - The wheels calibration is now automatically done when "vva_base_controller.launch" is launched and it detects that the calibration
       parameters file is not present. There is no need for a separate launch file for calibration.
     - The wheels PID controller implemented in the package "vva_base_controller" was modified, it now has two sets of coefficients: one for
       in place rotation and other for translation. This because the conditions of friction in these two situations are very different.
     - The node "vva_topicAnalyzerForAccelLimits.py" now calculates the acceleration limits and prints them in the screen.
       To invoke: "rosrun vva_nav_test vva_topicAnalyzerForAccelLimits.py"
     - The bash script "RunAsBackgroundScript/runVVA_ROSNodesOnBackground.sh" is now available to start all the nodes in the background.
       It has 4 options: "start_mapping", "start_navigation", "status" and "stop"
     - The node “vva_navigation/ vva_navigation_correction.py” was modified to stop the RP-Lidar when there is no any goal in progress.
     - A watchdog crontab script was done to monitor the status of the WiFi connection and restart the WiFi interface if the connectivity fails.
       Name of the script: "EmailNotificationAndWatchdogScripts/WiFiWatchdog.sh".
     - The package "vva_robot_healthcheck" was added to receive the status of the WiFi from the watchdog script and notify the user through the
       Kinect's LED and tilt.
     - The node "vva_user_intents/ vva_navigation_intent.py" was modified to transform all the goals to the "map" frame.
     
     - Pre-requisites:
          - Use the package depth_nav_tools/laserscan_kinect, it is not available in Melodic but it can be downloaded and compiled. It is found in:
            "$ROS_HOME_WS/ROSCompiledPackages/ROSCompiledPackages_ws/src/depth_nav_tools/laserscan_kinect/"
          - If you want to make a modification of the request or the response of the ROS Service used in the Mobile App, then rosjava is needed to
            generate the .jar file. It is found in: "$ROS_HOME_WS/ROSCompiledPackages/rosjava_minimal"
          - DeepSpeech v0.8.2 for TensorFlow-Lite installed over Python 3.7 (to install: "python3.7 -m pip install deepspeech-tflite")

	   - Included folders and files:
          20200225_CostosVehiculoVigilanciaAutonomo.ods - Summary of the costs of hardware used so far to build the vehicle
          20200524_Optimization_CliffDetector_LaserscanKinect.ods - Results of optimization tests of the parameters of the nodes vva_cliff_detector and
                                                                    laserscan_kinect.
          20200620_CalculosSeleccionNuevosMotoresYDiseñoHW.ods - Estimations of the required specifications for the new hardware: motors, batteries, etc
                                                                 proposals of different designs for the new hardware and estimations of the costs.
          20200725_TestsOfHARKAudioSourceLocation.ods - Tests of HARK library about the accuracy to identify the location of the source of the sound.
          20200729_TestsOfSpeechRecognitionModels.ods - Tests about the accuracy of Speech Recognition with DeepSpeech v0.6.1 (English) and
                                                        PocketSphinx 5 pre-alpha (Spanish).
          20201023_MapeoVelocidadVehiculo_ComandosMotor.ods - Calculations about the ticks of the wheels encoders and about the motor limits to turn
                                                              in curves (not in place rotations)
          Android - Contains the Android Studio project (VVA_Mobile_Client) of the Mobile App and a rosjava workspace (vva_rosjava_ws) to generate
                    the necessary .jar files to allow the Mobile App to communicate with the ROS service.
          Arduino - Contains the source code to be used in the microcontroller Arduino Mega2560, in charge of all the low level hardware control.
          ConfiguracionOpenVPN - Configuration files of OpenVPN for client and server
          DeepSpeechModule - Contains the DeepSpeech TensorFlow model files, the Python source code and bash scripts to run the Automatic Speech Recognition.
          Design - Design document and diagrams of VVA, including the software architecture.
          diagnosticos_tf_tree_y_rqt_graph - Images of diagrams of the TF-Tree and rqt_graph
          EmailNotificationAndWatchdogScripts - Crontab scripts in charge of email notifications of events and watchdog of the health of the system.
          EstadisticasControlPIDYAcelerac - Tabulated statistics and graphs of the PID controller to define the values of the coeficients Kp, Ki y Kd.
                                            Statistics to determine the acceleration limits of the vehicle for the configuration of the base_local_planner.
                                            Statistics of the quantity of loop closures reported by rtabmap in different light conditions.
          OtherScripts - Contains a script to prepare the datasets for training a DeepSpeech model and scripts to test motor encoders.
          Rtabmap_SavedMaps - Saved RTABMap .db files (maps) from different locations on different light conditions.
          RunAsBackgroundScript - Contains a bash script to start all the modules in background.
          VVA_ws/src/upload_files_to_robot.sh - Script to copy selected packages from the Laptop to the robot.
          ../Backup/create_backup.sh - Creates a backup upon its execution.
     
	   - Included ROS packages:
          vva_base_controller - Control of the wheels and communication with the Arduino Mega2560.
          vva_cliff_detector - This module is a fork of the code of depth_nav_tools to slightly change the behavior regarding the topics.
                               It is in charge of detecting holes or cliffs.
          vva_cliff_detector_layer - Fork of the code of depth_nav_tools to reduce the verbosty of the module. It is in charge of including
                                     the holes and cliffs in the costmap.
          vva_description - Contains the URDF model and the launch files for rviz.
          vva_gazebo - Gazebo worlds.
          vva_jnano_consolidated - Consolidated launch file that starts in the J-Nano: Base controller, OpenNI Kinect, kinect_aux, RP-Lidar, state publisher.
          vva_kinect_aux - In charge of changing the tilt of the Kinect and of publishing the position of the joints according with the
                           Kinect's IMU.
          vva_lidar_filter - LaserScans filter used to reduce the angle covered by the laserscan_kinect and hence remove the noise generated
                             by the Kinect in a small section of the field of view.
          vva_lidar_odom - LaserScan based odometry, uses the icp_odometry module of rtabmap.
          vva_msgs - Package that contains the defintions of the customized messages and services used in VVA.
          vva_navigation - Contains the modules related to the mapping, localization and navigation stack, including rtabmap and move_base.
          vva_nav_test - Manual control of the robot based on the keyboard arrow keys and also by scripting. It also contains a node to
                         generate customized statistics in tables based on selected topics.
          vva_robot_healthcheck - Receives services and topics with reports of the health of the system and provides feedback to the user using
                                  the Kinect's LED or any other available means.
          vva_rplidar_ros - Fork of the rplidar_ros package. It is modified to add options that enable the change of the values of the Lidar intensities.
          vva_topics_sync - Package that uses message_filters to synchronize the image_raw and camera_info topics generated by the Kinect.
                            Useful when the Kinect is in a remote node.
          vva_user_intents - This package contains nodes in charge of the execution of actions identified during the Intent Recognition. Besides
                             is responsible for the high level behaviors of the robot.
          vva_voice_interact_client - This node is in charge of interfacing with the microphones array of the Kinect and perform the wake-word detection,
                                      which is based on Porcupine. This package is not compatible with J-Nano, see the "Known issues" section.
          vva_voice_interact_server - This package receives an audio clip through a ROS service and invokes the DeepSpeech module to perform the 
                                      Automatic Speech Recognition. It also performs the mapping between audio transcripts and intent services.
                                      
     - Known issues:
          - The model is not shown in Gazebo, when the joints are of type "revolute". For example, this happened with the joints between the
            Kinect and the base.
          - vva_cliff_detector doesn't work when simulated in Gazebo, it shows the error: "Could not perform stairs detection:
            Depth image has unsupported encoding: 32FC1"
          - Sometimes the depth image transmission has interruptions or it just doesn't arrive any more, hence the navigation stack
            stops working. The cause of this issue is unknown so far, maybe it is due to the weakness of the WiFi signal in some locations.
          - The package "vva_voice_interact_client" (previously called "vva_voice_interact_rbpi") didn't work on the J-Nano because the wake-word
            detection program, Porcupine, didn't have a compatible version for arm64 architecture.
            - When voice commands are captured using the microphones array of the Kinect, the Speech Recognition doesn't work accurately. Maybe
              due to the environmental noise and because the microphones are far from the person speaking.
          - When running on the J-Nano, the depth image of the Kinect doesn't work properly when the argument “depth_registration” is set to "true"
            when using the package "openni_launch". This didn't happen on the RbPi. This was avoided using other available topics.
          - The odometry based on Lidar (vva_lidar_odom) gets lost when the vehicle follows a curved trajectory (translation + rotation).
     
     - Commands to test it in simulation:
     
          # Deploy the model in Gazebo:
             roslaunch vva_gazebo vva_world.launch
          # Start rviz and the state_publishers:
             roslaunch vva_description rviz_rtabmap_simulation.launch
          # -------------------------------------------------------------------------------------------------------   
          # Mapping:
          # -------------------------------------------------------------------------------------------------------   
          # Start rtabmap:
             roslaunch vva_navigation vva_rtabmap_simulation.launch localization:=false
          # Use the teleop:
             roslaunch vva_nav_test vva_teleop3.launch
          # Move the robot to generate the map, turn 360° in different places,
          # In the Laptop: If you want to restart the map execute:
             rosservice call /rtabmap_jnano/reset
          # -------------------------------------------------------------------------------------------------------   
          # Once done, close rtabmap and the teleop.
          # Localization:
          # -------------------------------------------------------------------------------------------------------   
          # Start rtabmap:
             roslaunch vva_navigation vva_rtabmap_simulation.launch localization:=true
          # Start the modules: navigation (move_base), image-camera_info sync, laserscan_kinect, laserscan_kinect_filter and vva_navigation_correction:
             roslaunch vva_navigation vva_consolidated_nav.launch simulation:=true
          # Update the location coordinates in navigation_intent_params_simulation.yaml and start the navigation-intent node:
             roslaunch vva_user_intents vva_user_intents.launch simulation:=true
          # Start the DeepSpeech module:
             $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/DeepSpeechModule/run_DS_module_laptop.sh
          # Start the Voice Interaction Speech Recognition module:
             roslaunch vva_voice_interact_server vva_voice_interact_server.launch
     
     - Commands to test it in the hardware through SSH sessions (for Debugging):
     
          # In the J-Nano: If it is required, stop the RPLidar motor:
             # If not stared, start the RPLidar node:
               roslaunch vva_rplidar_ros vva_rplidar_ros.launch rplidar_port:=/dev/ttyUSB?
             # From any device (J-Nano or Laptop) call the service:
               rosservice call /stop_motor
             # To start the motor again, close the RPLidar node or call the service:
               rosservice call /start_motor
          # In the J-Nano: If it is required, to calibrate the wheels motors when vva_base_controller is started:
             rm /home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_base_controller/wheels_calibration/wheels_calibration.txt
          # -------------------------------------------------------------------------------------------------------
          # In the J-Nano: Update the /dev/ttyUSB? number. Start the modules: vva_base_controller, kinect, kinect_aux, rplidar and state_publishers:
             roslaunch vva_jnano_consolidated vva_jnano_lowlevel_consolidated.launch arduino_port:=/dev/ttyUSB? rplidar_port:=/dev/ttyUSB?
          # In the J-Nano: Start the odometry based on laserscan:
             roslaunch vva_lidar_odom vva_lidar_odom.launch
             
          # In the Laptop: (Optional) Start rviz:
             roslaunch vva_description rviz_rtabmap_hw.launch
          # -------------------------------------------------------------------------------------------------------   
          # Mapping:
          # -------------------------------------------------------------------------------------------------------
          # In the J-Nano: Start the rtabmap, in mapping mode:
             roslaunch vva_navigation vva_rtabmap_hw.launch localization:=false
             
          # In the Laptop: (Optional) Start the rtabmapviz:
             roslaunch vva_navigation vva_rtabmapviz_hw.launch
          # In the Laptop: Use the teleop:
             roslaunch vva_nav_test vva_teleop3.launch
          # Move the robot to generate the map, turn 360° in different places,
          # In the Laptop: If you want to restart the map execute:
             rosservice call /rtabmap_jnano/reset
          # -------------------------------------------------------------------------------------------------------   
          # Once done, close rtabmap and the teleop.
          # Localization:
          # -------------------------------------------------------------------------------------------------------
          # In the J-Nano: Start the rtabmap, in localization mode:
             roslaunch vva_navigation vva_rtabmap_hw.launch localization:=true
          # In the J-Nano: Start the modules: vva_odom_correction, move_base, vva_cliff_detector, laserscan_kinect and vva_navigation_correction:
             roslaunch vva_navigation vva_consolidated_nav.launch simulation:=false
          # In the J-Nano: Update the location coordinates in navigation_intent_params.yaml and launch the navigation-intent node:
             roslaunch vva_user_intents vva_user_intents.launch simulation:=false
          # In the J-Nano: Start the DeepSpeech module:
             $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/DeepSpeechModule/run_DS_module_jnano.sh
          # In the J-Nano: Start the Voice Interaction Speech Recognition module:
             roslaunch vva_voice_interact_server vva_voice_interact_server.launch
          # In the J-Nano: Start the Robot Health Check module:
             roslaunch vva_robot_healthcheck vva_robot_healthcheck.launch

     - Commands to test it in the hardware using background execution:

          # -------------------------------------------------------------------------------------------------------   
          # Mapping:
          # -------------------------------------------------------------------------------------------------------
          # In the J-Nano: Start the mapping modules:
             $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/RunAsBackgroundScript/runVVA_ROSNodesOnBackground.sh start_mapping

          # In the Laptop: Use the teleop:
             roslaunch vva_nav_test vva_teleop3.launch
          # Move the robot to generate the map,
          # In the Laptop: If you want to restart the map execute:
             rosservice call /rtabmap_jnano/reset
          # -------------------------------------------------------------------------------------------------------   
          # In the Laptop: (Optional) Start rviz:
             roslaunch vva_description rviz_rtabmap_hw.launch
          # -------------------------------------------------------------------------------------------------------   
          # Once done, close the teleop.
          # Localization:
          # -------------------------------------------------------------------------------------------------------
          # In the J-Nano: Start the navigation and voice recognition modules:
             $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/RunAsBackgroundScript/runVVA_ROSNodesOnBackground.sh start_navigation
          # In the J-Nano: (Optional) check the status of the launch files processes:
             $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/RunAsBackgroundScript/runVVA_ROSNodesOnBackground.sh status
          # In the J-Nano: (Optional) check the logs:
             cd ~/.ros/log/
             cd ~/.ros/log/latest/
          # In the J-Nano: (Optional) terminate all processes:
             $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/RunAsBackgroundScript/runVVA_ROSNodesOnBackground.sh stop

============================================================================================================================================
VehiculoVigilanciaAutonomo_v0.6.2:
     - Developed in the Sprint 6.
     - Intended to integrate with the new Web client.
     - From this version on, a Git local repository is used.
     - The launch files "vva_navigation/vva_rtabmap_*.launch" are now launched or stopped from the node "vva_web_server/vva_robot_management_node.py"
     - It now runs on ROS1 Noetic
     
     

     - Pre-requisites:
       - Compile web_video_server 
     

	   - Included folders and files:
     
	   - Included ROS packages:
                                      
     - Known issues:
       - rosbridge throws an error when using background execution, also when the mapping is stopped by pressing Done and then started again.
         The error happens when the movement arrows of the Web interface are presssed.
         The error reported: "Could not process inbound connection: [/rosbridge_websocket] is not a publisher of [/cmd_vel]."
     
     
     - Commands to test it in simulation:
     
          # Start the Node.JS server for the Web client:
             cd /home/theuser/AAData/Documents2/WebDevelopment/VVA_ManagementConsole/
             npm start
          # Start the modules to integrate with the Web client: rosbridge_server, web_video_server, robot_pose_publisher and vva_robot_management:
             roslaunch vva_web_server vva_web_server.launch \
                rtabmap_launch_file:="/home/theuser/AAData/Documents2/ROS/VehiculoVigilanciaAutonomo/VVA_ws/src/vva_navigation/launch/vva_rtabmap_simulation.launch" \
                nav_consolidated_launch_file:="/home/theuser/AAData/Documents2/ROS/VehiculoVigilanciaAutonomo/VVA_ws/src/vva_navigation/launch/vva_consolidated_nav.launch" \
                simulation:=true \
                2> >(grep -v 'TF_REPEATED_DATA \| line\ 278')
          # Deploy the model in Gazebo:
             roslaunch vva_gazebo vva_world.launch
          # Start rviz and the state_publishers:
             roslaunch vva_description rviz_rtabmap_simulation.launch 2> >(grep -v 'TF_REPEATED_DATA \| line\ 278')
          # Update the location coordinates in navigation_intent_params_simulation.yaml and start the navigation-intent node:
             roslaunch vva_user_intents vva_user_intents.launch simulation:=true 2> >(grep -v 'TF_REPEATED_DATA \| line\ 278')
          # -------------------------------------------------------------------------------------------------------   
          # Mapping:
          # -------------------------------------------------------------------------------------------------------
          # Press Start in the Web client, move the robot to generate the map, turn 360° in different places,
          # press Done.
          # -------------------------------------------------------------------------------------------------------   
          # Localization:
          # -------------------------------------------------------------------------------------------------------   
          # TODO: Do the procedure from the Web client
             
          # (Optional) Start the DeepSpeech module:
             $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/DeepSpeechModule/run_DS_module_laptop.sh
          # (Optional) Start the Voice Interaction Speech Recognition module:
             roslaunch vva_voice_interact_server vva_voice_interact_server.launch
             
     
     - Commands to test it in the hardware through SSH sessions (for Debugging):
     
          # In the J-Nano: If it is required, stop the RPLidar motor:
             # If not stared, start the RPLidar node:
               roslaunch vva_rplidar_ros vva_rplidar_ros.launch rplidar_port:=/dev/ttyUSB?
             # From any device (J-Nano or Laptop) call the service:
               rosservice call /stop_motor
             # To start the motor again, close the RPLidar node or call the service:
               rosservice call /start_motor
          # In the J-Nano: If it is required, to calibrate the wheels motors when vva_base_controller is started:
             rm $ROS_HOME_WS/VehiculoVigilanciaAutonomo/VVA_ws/src/vva_base_controller/wheels_calibration/wheels_calibration.txt
          # In the J-Nano: Update the ROS_IP and ROS_MASTER_URI env vars:
             vim ~/.bashrc
             source ~/.bashrc
          # In the J-Nano: Determine the ttyUSB port numbers for Arduino and RPLidar:
             TTYUSB0_PATH=`find /sys/devices -name ttyUSB0 | head -1 | cut -d/ -f-7`; cat $TTYUSB0_PATH/idVendor
             # idVendor Arduino: 0403
             # idVendor RPLidar: 10c4
          # -------------------------------------------------------------------------------------------------------
          # In the Laptop: Start the Node.JS server for the Web client:
             cd /home/theuser/AAData/Documents2/WebDevelopment/VVA_ManagementConsole/
             npm start

          # In the J-Nano: Start the modules to integrate with the Web client: rosbridge_server, web_video_server, robot_pose_publisher and vva_robot_management:
             roslaunch vva_web_server vva_web_server.launch \
                rtabmap_launch_file:="/home/ubuntu/ROS/VehiculoVigilanciaAutonomo/VVA_ws/src/vva_navigation/launch/vva_rtabmap_hw.launch" \
                nav_consolidated_launch_file:="/home/ubuntu/ROS/VehiculoVigilanciaAutonomo/VVA_ws/src/vva_navigation/launch/vva_consolidated_nav.launch" \
                simulation:=false
          # In the J-Nano: Update the /dev/ttyUSB? number. Start the modules: vva_base_controller, kinect, kinect_aux, rplidar and state_publishers:
             roslaunch vva_jnano_consolidated vva_jnano_lowlevel_consolidated.launch arduino_port:=/dev/ttyUSB? rplidar_port:=/dev/ttyUSB?
          # In the J-Nano: Start the odometry based on laserscan:
             roslaunch vva_lidar_odom vva_lidar_odom.launch
             
          # In the Laptop: (Optional) Start rviz:
             roslaunch vva_description rviz_rtabmap_hw.launch
          # -------------------------------------------------------------------------------------------------------   
          # Mapping:
          # -------------------------------------------------------------------------------------------------------
          # In the Laptop: (Optional) Start the rtabmapviz:
             roslaunch vva_navigation vva_rtabmapviz_hw.launch

          # Press Start in the Web client, move the robot to generate the map, turn 360° in different places,
          # press Done.
          # -------------------------------------------------------------------------------------------------------   
          # Localization:
          # -------------------------------------------------------------------------------------------------------

          # MIGRADO --- In the J-Nano: MANAGED FROM vva_robot_management_node.py --  Start the rtabmap, in localization mode:
             roslaunch vva_navigation vva_rtabmap_hw.launch localization:=true
          # MIGRADO --- In the J-Nano: Start the modules: vva_odom_correction, move_base, vva_cliff_detector, laserscan_kinect and vva_navigation_correction:
             roslaunch vva_navigation vva_consolidated_nav.launch simulation:=false


          # In the J-Nano: Update the location coordinates in navigation_intent_params.yaml and launch the navigation-intent node:
             roslaunch vva_user_intents vva_user_intents.launch simulation:=false
          # In the J-Nano: Start the Robot Health Check module:
             roslaunch vva_robot_healthcheck vva_robot_healthcheck.launch


          # NO APLICA --- In the J-Nano: Start the DeepSpeech module:
             $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/DeepSpeechModule/run_DS_module_jnano.sh
          # NO APLICA --- In the J-Nano: Start the Voice Interaction Speech Recognition module:
             roslaunch vva_voice_interact_server vva_voice_interact_server.launch



     - Commands to test it in the hardware using background execution:
     
          # In the J-Nano: If it is required, to calibrate the wheels motors when vva_base_controller is started:
             rm $ROS_HOME_WS/VehiculoVigilanciaAutonomo/VVA_ws/src/vva_base_controller/wheels_calibration/wheels_calibration.txt
          # In the J-Nano: Update the ROS_IP and ROS_MASTER_URI env vars:
             vim ~/.bashrc
             source ~/.bashrc
          # -------------------------------------------------------------------------------------------------------
          # In the Laptop: Start the Node.JS server for the Web client:
             cd /home/theuser/AAData/Documents2/WebDevelopment/VVA_ManagementConsole/
             npm start
          # -------------------------------------------------------------------------------------------------------   
          # Mapping:
          # -------------------------------------------------------------------------------------------------------
          # In the J-Nano: Start the mapping modules:
             $ROS_HOME_WS/VehiculoVigilanciaAutonomo/RunAsBackgroundScript/runVVA_ROSNodesOnBackground.sh start_mapping

          # Press Start in the Web client, move the robot to generate the map, turn 360° in different places,
          # press Done.
          # -------------------------------------------------------------------------------------------------------
          # In the Laptop: (Optional) Start rviz:
             roslaunch vva_description rviz_rtabmap_hw.launch
          # -------------------------------------------------------------------------------------------------------
          # Localization:
          # -------------------------------------------------------------------------------------------------------

          # In the J-Nano: -- PENDING TO MIGRATE - INCLUDE vva_robot_management_node.py --  Start the navigation and voice recognition modules:
             $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/RunAsBackgroundScript/runVVA_ROSNodesOnBackground.sh start_navigation
             
          # -------------------------------------------------------------------------------------------------------   
          # Troubleshooting:
          # -------------------------------------------------------------------------------------------------------
          # In the J-Nano: (Optional) check the status of the launch files processes:
             $ROS_HOME_WS/VehiculoVigilanciaAutonomo/RunAsBackgroundScript/runVVA_ROSNodesOnBackground.sh status
          # In the J-Nano: (Optional) check the logs:
             cd ~/.ros/log/
             cd ~/.ros/log/latest/
          # In the J-Nano: (Optional) terminate all processes:
             $ROS_HOME_WS/VehiculoVigilanciaAutonomo/RunAsBackgroundScript/runVVA_ROSNodesOnBackground.sh stop

             



















