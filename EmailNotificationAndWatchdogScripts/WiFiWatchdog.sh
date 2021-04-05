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
# andres.arboleda AT gmail DOT com, (november/2020)
#------------------------------------------------------------------------------

# Watchdog script to monitor the WiFi connection. If it fails, this script
# tries to restart the connection. If the connection is not restablished
# a ROS service is called to indicate that the WiFi connection is down.

WIFI_STATUS_FILENAME=/home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6.1/EmailNotificationAndWatchdogScripts/WiFiWatchdog_PrevStatus.txt

REPORT_WIFI_OK_SERVICE=/vva_robot_healthcheck_node/report_wifi_ok
REPORT_WIFI_ERROR_SERVICE=/vva_robot_healthcheck_node/report_wifi_error


# Setup the ROS environment
source /opt/ros/melodic/setup.bash

DATE_AND_HOUR=`date "+%Y%m%d_%H%M%S"`

# Test WiFi connectivity
DEFAULT_GW_IP=`ip r | grep default | awk '{print $3}'`
if [ "$DEFAULT_GW_IP" != "" ]; then
    PACKETS_RECEIVED=`ping -c 5 -q $DEFAULT_GW_IP | grep 'packets transmitted' | awk '{print $4}'`
    if [ "$PACKETS_RECEIVED" -gt 2 ]; then
        if [ -f $WIFI_STATUS_FILENAME ]; then
            OLD_STATUS=`head -n 1 $WIFI_STATUS_FILENAME`
            if [ "$OLD_STATUS" = "ok" ]; then
                echo "$DATE_AND_HOUR, The status has not changed."
                exit 0
            fi
        fi
        echo "ok" > $WIFI_STATUS_FILENAME
        rosservice call $REPORT_WIFI_OK_SERVICE
        echo "$DATE_AND_HOUR, WiFi connectivity ok."
        exit 0
    fi
fi
    
# Restart the WiFi connection
echo "$DATE_AND_HOUR, WiFi connectivity error. Restarting WiFi (this will take 15 seconds)..."
nmcli radio wifi off
sleep 5
nmcli radio wifi on
sleep 10

DATE_AND_HOUR=`date "+%Y%m%d_%H%M%S"`

# Test again WiFi connectivity
DEFAULT_GW_IP=`ip r | grep default | awk '{print $3}'`
if [ "$DEFAULT_GW_IP" != "" ]; then
    PACKETS_RECEIVED=`ping -c 5 -q $DEFAULT_GW_IP | grep 'packets transmitted' | awk '{print $4}'`
    if [ "$PACKETS_RECEIVED" -gt 2 ]; then
        if [ -f $WIFI_STATUS_FILENAME ]; then
            OLD_STATUS=`head -n 1 $WIFI_STATUS_FILENAME`
            if [ "$OLD_STATUS" = "ok" ]; then
                echo "$DATE_AND_HOUR, The status has not changed."
                exit 0
            fi
        fi
        echo "ok" > $WIFI_STATUS_FILENAME
        rosservice call $REPORT_WIFI_OK_SERVICE
        echo "$DATE_AND_HOUR, WiFi connectivity ok."
        exit 0
    fi
fi

if [ -f $WIFI_STATUS_FILENAME ]; then
    OLD_STATUS=`head -n 1 $WIFI_STATUS_FILENAME`
    if [ "$OLD_STATUS" = "error" ]; then
        echo "$DATE_AND_HOUR, The status has not changed."
        exit 1
    fi
fi

# Call service to report error in WiFi
echo "$DATE_AND_HOUR, WiFi connectivity could not be fixed."
echo "error" > $WIFI_STATUS_FILENAME
rosservice call $REPORT_WIFI_ERROR_SERVICE
exit 1



