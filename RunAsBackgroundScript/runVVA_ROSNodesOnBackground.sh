#!/bin/bash

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
# andres.arboleda AT gmail DOT com, (october/2020)
#
# Modified by:
# andres.arboleda AT gmail DOT com, (april/2021)
#------------------------------------------------------------------------------

# This script starts the launch files of VVA in background.
# If the argument "start_mapping" is given, it starts only:
# - vva_web_server.launch (rosbridge_server, web_video_server, robot_pose_publisher, vva_robot_management)
# - vva_jnano_lowlevel_consolidated.launch (vva_base_controller, kinect, kinect_aux, rplidar and state_publishers)
# - vva_lidar_odom.launch
# - vva_robot_healthcheck
#
#
# PENDING TO MODIFY:
# If the argument "start_navigation" is given, it starts the full navigation stack and voice recognition services:
# - vva_jnano_lowlevel_consolidated.launch (vva_base_controller, kinect, kinect_aux, rplidar and state_publishers)
# - vva_lidar_odom.launch
# - vva_rtabmap_hw.launch (in localization mode)
# - vva_consolidated_nav.launch (vva_odom_correction, move_base, vva_cliff_detector, laserscan_kinect and vva_navigation_correction)
# - vva_user_intents.launch
# - DeepSpeechModule
# - vva_voice_interact_server.launch
# - vva_robot_healthcheck

ROS_LOG_DIR=~/.ros/log
DATE_AND_HOUR=`date "+%Y%m%d_%H%M%S"`

LIST_OF_PROCESSES=( vva_web_server.launch \
                    vva_jnano_lowlevel_consolidated.launch \
                    vva_lidar_odom.launch \
                    vva_consolidated_nav.launch \
                    vva_user_intents.launch \
                    vva_deepspeech_v0.8.2_server_main.py \
                    vva_voice_interact_server.launch \
                    vva_robot_healthcheck.launch )


case "$1" in
    start_mapping)
        # Determine which ttyUSB is used by Arduino and which by RPLidar
        TTYUSB0_PATH=`find /sys/devices -name ttyUSB0 | head -1 | cut -d/ -f-7`
        TTYUSB0_VENDOR=`cat $TTYUSB0_PATH/idVendor`
        # idVendor Arduino: 0403
        # idVendor RPLidar: 10c4
        if [ "$TTYUSB0_VENDOR" = "0403" ]; then
            ARDUINO_PORT=/dev/ttyUSB0
            RPLIDAR_PORT=/dev/ttyUSB1
        else    
            ARDUINO_PORT=/dev/ttyUSB1
            RPLIDAR_PORT=/dev/ttyUSB0
        fi
        
        PROCESS_NUMBER=`ps -ef | grep vva_web_server.launch | grep -v grep | wc -l`
        if [ "$PROCESS_NUMBER" -gt 0 ]; then
            echo "ROS processes are already running, please stop all ROS processes. Type: ./runVVA_ROSNodesOnBackground.sh stop"
            exit 1
        fi
        

        # Start the vva_web_server, lowlevel_consolidated and lidar_odom launch files
        # If the ROS log directory exists, launch the files and create logs
        if [ -d "$ROS_LOG_DIR" ]; then
            roslaunch vva_web_server vva_web_server.launch \
                   rtabmap_launch_file:="/home/ubuntu/ROS/VehiculoVigilanciaAutonomo/VVA_ws/src/vva_navigation/launch/vva_rtabmap_hw.launch" \
                   > $ROS_LOG_DIR/vva_web_server_$DATE_AND_HOUR.log 2>&1 &
                   
        # If the ROS logs were purged, the log directory wouldn't exists, then, launch the files but this time don't create logs
        else
            roslaunch vva_web_server vva_web_server.launch \
                   rtabmap_launch_file:="/home/ubuntu/ROS/VehiculoVigilanciaAutonomo/VVA_ws/src/vva_navigation/launch/vva_rtabmap_hw.launch" \
                   > /dev/null 2>&1 &
        fi
        echo "Starting vva_web_server.launch..."
        sleep 5

        # ROS_LOG_DIR was automatically created in previous step
        roslaunch vva_jnano_consolidated vva_jnano_lowlevel_consolidated.launch arduino_port:=$ARDUINO_PORT rplidar_port:=$RPLIDAR_PORT \
               > $ROS_LOG_DIR/vva_jnano_lowlevel_consolidated_$DATE_AND_HOUR.log 2>&1 &
        echo "Starting vva_jnano_lowlevel_consolidated.launch..."
        sleep 7
        
        roslaunch vva_lidar_odom vva_lidar_odom.launch > $ROS_LOG_DIR/vva_lidar_odom_$DATE_AND_HOUR.log 2>&1 &
        echo "Starting vva_lidar_odom.launch..."
        sleep 5

#        roslaunch vva_navigation vva_rtabmap_hw.launch localization:=false > $ROS_LOG_DIR/vva_rtabmap_hw_$DATE_AND_HOUR.log 2>&1 &
#        echo "Starting vva_rtabmap_hw.launch in mapping mode..."
#        sleep 5
        
        roslaunch vva_robot_healthcheck vva_robot_healthcheck.launch > $ROS_LOG_DIR/vva_robot_healthcheck_$DATE_AND_HOUR.log 2>&1 &
        echo "Starting vva_robot_healthcheck.launch in mapping mode..."
        ;;
        

    start_navigation)
        # Determine which ttyUSB is used by Arduino and which by RPLidar
        TTYUSB0_PATH=`find /sys/devices -name ttyUSB0 | head -1 | cut -d/ -f-7`
        TTYUSB0_VENDOR=`cat $TTYUSB0_PATH/idVendor`
        # idVendor Arduino: 0403
        # idVendor RPLidar: 10c4
        if [ "$TTYUSB0_VENDOR" = "0403" ]; then
            ARDUINO_PORT=/dev/ttyUSB0
            RPLIDAR_PORT=/dev/ttyUSB1
        else    
            ARDUINO_PORT=/dev/ttyUSB1
            RPLIDAR_PORT=/dev/ttyUSB0
        fi
        
        PROCESS_NUMBER=`ps -ef | grep vva_jnano_lowlevel_consolidated.launch | grep -v grep | wc -l`
        if [ "$PROCESS_NUMBER" -eq 0 ]; then
            # Start the lowlevel_consolidated, lidar_odom, rtabmap (in navigation mode) and the other launch files
            # If the ROS log directory exists, launch the files and create logs
            if [ -d "$ROS_LOG_DIR" ]; then
                roslaunch vva_jnano_consolidated vva_jnano_lowlevel_consolidated.launch arduino_port:=$ARDUINO_PORT rplidar_port:=$RPLIDAR_PORT \
                       > $ROS_LOG_DIR/vva_jnano_lowlevel_consolidated_$DATE_AND_HOUR.log 2>&1 &
                       
            # If the ROS logs were purged, the log directory wouldn't exists, then, launch the files but this time don't create logs
            else
                roslaunch vva_jnano_consolidated vva_jnano_lowlevel_consolidated.launch arduino_port:=$ARDUINO_PORT rplidar_port:=$RPLIDAR_PORT \
                       > /dev/null 2>&1 &
            fi
            echo "Starting vva_jnano_lowlevel_consolidated.launch..."
            sleep 7
        fi
        
        # After launching the first file, ROS_LOG_DIR is automatically created
        PROCESS_NUMBER=`ps -ef | grep vva_lidar_odom.launch | grep -v grep | wc -l`
        if [ "$PROCESS_NUMBER" -eq 0 ]; then
            roslaunch vva_lidar_odom vva_lidar_odom.launch > $ROS_LOG_DIR/vva_lidar_odom_$DATE_AND_HOUR.log 2>&1 &
            echo "Starting vva_lidar_odom.launch..."
            sleep 5
        fi
        
        PROCESS_ID=`ps -ef | grep vva_rtabmap_hw.launch | grep -v grep | awk '{print $2}'`
        if [ "$PROCESS_ID" != "" ]; then
            kill $PROCESS_ID
            echo "Stopping vva_rtabmap_hw.launch..."
            sleep 7
        fi
        roslaunch vva_navigation vva_rtabmap_hw.launch localization:=true > $ROS_LOG_DIR/vva_rtabmap_hw_$DATE_AND_HOUR.log 2>&1 &
        echo "Starting vva_rtabmap_hw.launch in localization mode..."
        sleep 7
        
        PROCESS_NUMBER=`ps -ef | grep vva_consolidated_nav.launch | grep -v grep | wc -l`
        if [ "$PROCESS_NUMBER" -eq 0 ]; then
            roslaunch vva_navigation vva_consolidated_nav.launch simulation:=false > $ROS_LOG_DIR/vva_consolidated_nav_$DATE_AND_HOUR.log 2>&1 &
            echo "Starting vva_consolidated_nav.launch..."
            sleep 5
        fi
        
        PROCESS_NUMBER=`ps -ef | grep vva_user_intents.launch | grep -v grep | wc -l`
        if [ "$PROCESS_NUMBER" -eq 0 ]; then
            roslaunch vva_user_intents vva_user_intents.launch simulation:=false > $ROS_LOG_DIR/vva_user_intents_$DATE_AND_HOUR.log 2>&1 &
            echo "Starting vva_user_intents.launch..."
            sleep 3
        fi
        
        PROCESS_NUMBER=`ps -ef | grep vva_deepspeech_v0.8.2_server_main.py | grep -v grep | wc -l`
        if [ "$PROCESS_NUMBER" -eq 0 ]; then
            python3.7 $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/DeepSpeechModule/src/vva_deepspeech_v0.8.2_server_main.py \
              --audio $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_voice_interact_server/records/rcvd_temp.wav \
              --model $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/DeepSpeechModule/english_model_v0.8.2 \
              --port 39567 \
              > $ROS_LOG_DIR/DeepSpeechModule_$DATE_AND_HOUR.log 2>&1 &
            echo "Starting vva_deepspeech_v0.8.2_server_main.py..."
            sleep 3
        fi
          
        PROCESS_NUMBER=`ps -ef | grep vva_voice_interact_server.launch | grep -v grep | wc -l`
        if [ "$PROCESS_NUMBER" -eq 0 ]; then
            roslaunch vva_voice_interact_server vva_voice_interact_server.launch > $ROS_LOG_DIR/vva_voice_interact_server_$DATE_AND_HOUR.log 2>&1 &
            echo "Starting vva_voice_interact_server.launch..."
            sleep 3
        fi
        
        PROCESS_NUMBER=`ps -ef | grep vva_robot_healthcheck.launch | grep -v grep | wc -l`
        if [ "$PROCESS_NUMBER" -eq 0 ]; then
            roslaunch vva_robot_healthcheck vva_robot_healthcheck.launch > $ROS_LOG_DIR/vva_robot_healthcheck_$DATE_AND_HOUR.log 2>&1 &
            echo "Starting vva_robot_healthcheck.launch..."
        fi
        ;;
        
        
    status)
        for i in "${LIST_OF_PROCESSES[@]}"
        do
            PROCESS_ID=`ps -ef | grep $i | grep -v grep | awk '{print $2}'`
            if [ "$PROCESS_ID" != "" ]; then
              echo "$i...................Running"
            else
              echo "$i...................Stopped"
            fi
        done                    
        ;;
        
        
    stop)
        for i in "${LIST_OF_PROCESSES[@]}"
        do
            PROCESS_ID=`ps -ef | grep $i | grep -v grep | awk '{print $2}'`
            if [ "$PROCESS_ID" != "" ]; then
              kill $PROCESS_ID
            fi
        done                    
        ;;
        
        
    *)
        echo ""
        echo "Starts or stops all the ROS nodes of VVA."
        echo "Usage:  runVVA_ROSNodesOnBackground.sh start_mapping|start_navigation|status|stop"
        echo ""
        exit 1
        ;;
esac

exit 0










