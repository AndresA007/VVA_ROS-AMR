#!/bin/bash

python3 $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/DeepSpeechModule/src/vva_deepspeech_v0.8.2_server_main.py \
  --audio $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/VVA_ws/src/vva_voice_interact_server/records/rcvd_temp.wav \
  --model $ROS_HOME_WS/VehiculoVigilanciaAutonomo_v0.6.1/DeepSpeechModule/english_model_v0.8.2 \
  --port 39567

