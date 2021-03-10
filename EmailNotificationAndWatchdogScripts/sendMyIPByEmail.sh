#!/bin/bash

# Script de prueba para enviar la dirección IPv4 de la J-Nano cada vez que esta cambia

IfName="wlan0"
IpFileName="/home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6.1/EmailNotificationAndWatchdogScripts/myOldIPAddress.txt"


Mi_Dir_IP=`ip -br -4 a | grep $IfName | awk '{print $3}' | cut -d '/' -f 1`

if [ -f $IpFileName ]; then

    Old_IP_addr=$(head -n 1 $IpFileName)

    if [ "$Mi_Dir_IP" = "$Old_IP_addr" ]; then
	    echo "La dirección IP no ha cambiado."
        exit 1
    fi
    
fi

echo $Mi_Dir_IP > $IpFileName

Email_Subject="J-Nano: Mi dirección IP"
Email_Body="Mi dirección IP es: ${Mi_Dir_IP}<br><br>Att,<br>J-Nano"
/home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6.1/EmailNotificationAndWatchdogScripts/sendEmail.py "$Email_Subject" "$Email_Body"
