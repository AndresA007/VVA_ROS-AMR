#!/usr/bin/env python

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
# andres.arboleda AT gmail DOT com, (sep/2020)
# Modified by:
# andres.arboleda AT gmail DOT com, (nov/2020)
#------------------------------------------------------------------------------

import rospy
import os
import time
import socket
import numpy as np
from vva_msgs.srv import VVARecognizeSpeech, VVAVoiceCommandIntent
from pydub import AudioSegment, effects
from std_msgs.msg import UInt16 #, Float64


# Receives a wav file through a ROS service and calls the non-ROS DeepSpeech
# module through a TCP socket to perform Speech Recognition
class SpeechRecognition:
  def __init__(self):
    rospy.init_node('vva_speech_recognition_node')
    
    # Subscribed topics
    # ~ self.goal_status_sub = rospy.Subscriber('vva_navigation_correction/status', NavCorrectionStatus, self.goal_status_topic_callback)
    
    # Published topics
    self.kinect_led_option_pub = rospy.Publisher('vva_kinect/led_option', UInt16, queue_size=10)
    # ~ self.kinect_tilt_angle_pub = rospy.Publisher('vva_kinect/tilt_angle', Float64, queue_size=10)

    # Published services
    self.vva_recognize_speech_service = rospy.Service('/vva_recognize_speech', VVARecognizeSpeech, self.vva_recognize_speech_callback)

    # Parameters
    self.rate                   = rospy.get_param('~rate', 10)
    self.deepspeech_socket_port = rospy.get_param('~deepspeech_socket_port', 39567)
    self.wav_save_path          = rospy.get_param('~wav_save_path', os.path.join(os.path.dirname(__file__), '../records/') )
    self.received_audio_file_name = rospy.get_param('~received_audio_file_name', os.path.join(os.path.dirname(__file__), '../records/') + "rcvd_temp.wav")
    self.transcripts_mapping    = rospy.get_param('~transcripts_mapping')
    
    # Global variables for subscribed topics
    # ~ self.topic_name = None  
    
    # Other global variables
    self.deepspeech_socket_message = "start"
    self.pending_to_call_intent_service = False
    self.intent_service = ""
    self.intent_service_arg = ""
    self.kinect_led_start_time = rospy.Time.now()
    self.pending_normalize_kinect_led = False


  # ==================================================
  # Topics and services callback functions
  # ==================================================
  def vva_recognize_speech_callback(self,req):
    rospy.loginfo('vva_speech_recognition_node: Service request received with audio data from "%s"', req.client_id)
    
    if len(req.audio_data) == 0:
      rospy.loginfo('vva_speech_recognition_node: Audio data is empty.')
      return "ERROR_NO_AUDIO_DATA_RECEIVED", ""

    # Publish topics asking the Kinect to turn the LED off
    kinect_led_val = UInt16()
    kinect_led_val.data = 0      # off
    self.kinect_led_option_pub.publish(kinect_led_val)

    # If the audio comes from the Kinect mounted in the robot, save a copy of the wav file
    # for future construction of training data-sets.
    if req.client_id == "kinect":
      copy_filename = self.wav_save_path + "/" + time.strftime("%Y%m%d_%H%M%S") + ".wav"
      with open(copy_filename, 'wb') as copy_wav_file:
        copy_wav_file.write(req.audio_data)
      rospy.loginfo('vva_speech_recognition_node: A copy of the .wav file was saved at "%s"', copy_filename)
    
    with open(self.received_audio_file_name, 'wb') as rcvd_file:
      rcvd_file.write(req.audio_data)
      
    ## DEBUG
    # ~ rospy.loginfo('vva_speech_recognition_node: Temp audio file saved in: %s', self.received_audio_file_name)
    
    concat_transcripts = ""
    
    # Send the notification to the DeepSpeech module to perform DeepSpeech's
    # Speech To Text over the received audio file
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
      sock.connect(('localhost', int(self.deepspeech_socket_port)))
      sock.send("start")
      concat_transcripts = sock.recv(1024)
      sock.close()
    except Exception as e:
      rospy.logwarn("vva_speech_recognition_node: Couldn't connect to the DeepSpeech socket, make sure the DeepSpeech module is running: " + str(e))
          
    rospy.loginfo('vva_speech_recognition_node: Concatenated transcript: "%s"', concat_transcripts)
      
    # Perform a basic intent recognition over the concatenated transcripts
    status = self.intent_recognition(concat_transcripts)
    
    # Change the LED option of the Kinect indicating if the intent was recognized
    if status.startswith("INTENT_ACCEPTED"):
      kinect_led_val.data = 1      # green-hold
      rospy.loginfo('vva_speech_recognition_node: Intent recognition status "%s". Setting the LED in GREEN', status)
    else:
      kinect_led_val.data = 2      # red
      rospy.loginfo('vva_speech_recognition_node: Intent recognition status "%s". Setting the LED in RED', status)
    # Publish topic asking the Kinect to change the status of the LED
    self.kinect_led_option_pub.publish(kinect_led_val)
    # Set the timer to maintain the status of the LED before returning to normal
    self.kinect_led_start_time = rospy.Time.now()
    self.pending_normalize_kinect_led = True

    return status, concat_transcripts


  # ==================================================
  # Intent recognition
  # ==================================================
  def intent_recognition(self, text):
    
    # Iterate over each search string pattern (key) and service name
    for service_map in self.transcripts_mapping:
      search_key = service_map['key']
      search_key_sections = search_key.split('*')
      
      # If the value of the key that is before the '*' (if it has one) is found in the transcript
      if text.find(search_key_sections[0]) != -1:
        
        # If the key has a '*'
        if len(search_key_sections) > 1:
          # Extract the value that goes instead of the '*' and afer it, to pass it to the service call
          self.intent_service_arg = text.partition(search_key_sections[0])[2]
        else:
          # The search key doesn't have a '*', hence only pass an empty string
          self.intent_service_arg = ""
          
        self.intent_service = service_map['service']
        self.pending_to_call_intent_service = True
        
        status = "INTENT_ACCEPTED_" + service_map['service']
        break
        
      else:
        status = "ERROR_INTENT_NOT_RECOGNIZED"
      
    return status



  # ==================================================
  # Update function
  # ==================================================
  def update(self):

    if self.pending_to_call_intent_service:
      # Call the intent service
      try:
        rospy.loginfo("vva_speech_recognition_node: Waiting for %s service to be started...", self.intent_service)
        rospy.wait_for_service(self.intent_service, timeout=10)
      except rospy.ROSException as e:
        rospy.logwarn("vva_speech_recognition_node: Service not available: " + str(e))

      try:
        rospy.loginfo("vva_speech_recognition_node: Calling intent service %s", self.intent_service)
        srv_intent_execution = rospy.ServiceProxy(self.intent_service, VVAVoiceCommandIntent)
        
        srv_response = srv_intent_execution(self.intent_service_arg)
          
        rospy.loginfo('vva_speech_recognition_node: Intent service response: %s', srv_response)
      except rospy.ServiceException as e:
        rospy.logwarn("vva_speech_recognition_node: Service failed: " + str(e))

      self.pending_to_call_intent_service = False


    # Count time showing the Kinect LED status then return it to the normal green blinkin state
    if (rospy.Time.now() - self.kinect_led_start_time).to_sec() > 2 and self.pending_normalize_kinect_led:
      kinect_led_val = UInt16()
      kinect_led_val.data = 4      # blinking green
      self.kinect_led_option_pub.publish(kinect_led_val)
      
      self.pending_normalize_kinect_led = False



  # ==================================================
  # Main loop
  # ==================================================
  def spin(self):
    rospy.loginfo("vva_speech_recognition_node: Start")
    rate = rospy.Rate(self.rate)
    rospy.on_shutdown(self.shutdown)
    
    ## DEBUG
    # ~ rospy.loginfo("vva_speech_recognition_node: transcripts_mapping: " + str(self.transcripts_mapping))
    
    rospy.loginfo("vva_speech_recognition_node: Waiting for audio file from the ROS service client...")
    
    while not rospy.is_shutdown():
      self.update()
      rate.sleep()
    rospy.spin()


  # ==================================================
  # If shutdown
  # ==================================================

  def shutdown(self):
    rospy.loginfo("vva_speech_recognition_node: Stop. CHAO PESCAO!")
    
    # Send an exit notification to the DeepSpeech module, so it can close the socket
    # ~ sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # ~ try:
      # ~ sock.connect(('localhost', int(self.deepspeech_socket_port)))
      # ~ sock.send("exit")
      # ~ sock.close()
    # ~ except Exception as e:
      # ~ rospy.loginfo("vva_speech_recognition_node: Couldn't connect to the DeepSpeech socket, make sure the DeepSpeech module is running: " + str(e))
    
    # Stop message
    rospy.sleep(1)
    

# ==================================================
# Main
# ==================================================
def main():
  speech_recognition = SpeechRecognition()
  speech_recognition.spin()

if __name__ == '__main__':
  main()









