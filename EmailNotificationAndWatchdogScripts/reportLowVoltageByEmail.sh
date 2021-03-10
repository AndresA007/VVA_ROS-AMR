#!/bin/bash

# Script de prueba para enviar un email cuando el syslog reporta muchos eventos de baja de voltaje
# Se corre a las :10 de cada hora, excepto las 00, y revisa cuantos eventos hubo en la hora anterior.

# Notify the Under-voltage events
# 10 1-23 * * * /home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6.1/EmailNotificationAndWatchdogScripts/reportLowVoltageByEmail.sh > /home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6.1/EmailNotificationAndWatchdogScripts/reportLowVoltageByEmail.status


UmbralDeEventos=10


CurrentMonthDay=`date "+%b %e"`
CurrentHour=`date "+%k"`
CurrentHourMinusOne=$(( $CurrentHour - 1 ))

GrepDate="$CurrentMonthDay $CurrentHourMinusOne:"

#echo "tail -5000 /var/log/syslog | grep "$GrepDate" | grep 'Under-voltage detected\|Voltage normalised' | wc -l"

NumberOfEvents=`tail -5000 /var/log/syslog | grep "$GrepDate" | grep 'Under-voltage detected\|Voltage normalised' | wc -l`

if [ $NumberOfEvents -lt $UmbralDeEventos ]; then
	    echo "Se detectaron $NumberOfEvents eventos de Under-voltage para la hora $GrepDate. Es menor que el umbral: $UmbralDeEventos. No se envía notificación."
        exit 1
fi

Email_Subject="VVA: Bajo voltaje detectado"
Email_Body="Se detectaron $NumberOfEvents eventos de voltaje bajo en la hora: $GrepDate.<br><br>Att,<br>VVA"
/home/ubuntu/ROS/VehiculoVigilanciaAutonomo_v0.6.1/EmailNotificationAndWatchdogScripts/sendEmail.py "$Email_Subject" "$Email_Body"


