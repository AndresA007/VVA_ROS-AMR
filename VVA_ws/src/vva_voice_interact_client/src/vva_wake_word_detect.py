#!/usr/bin/env python

#------------------------------------------------------------------------------
# Copyright 2018 Picovoice Inc.
# Copyright (c) 2020, Andres A. <andres.arboleda AT gmail DOT com>
# All rights reserved.
#
# License: Apache License, version 2.0
# License copy in: http://www.apache.org/licenses/
# 
# Based on the code of "porcupine_demo_mic.py" published by:
# Alireza Kenarsari, Picovoice Inc.
# 
# Github repo of original code:
# https://github.com/Picovoice/porcupine
# 
# This version modified by:
# andres.arboleda AT gmail DOT com, (july/2020)
#------------------------------------------------------------------------------


import rospy
import os
import struct
import sys
import pyaudio
import wave
from datetime import datetime
from vva_msgs.srv import VVARecognizeSpeech
from std_msgs.msg import UInt16
from std_msgs.msg import Float64

sys.path.append(os.path.join(os.path.dirname(__file__), '../porcupine/binding/python'))
sys.path.append(os.path.join(os.path.dirname(__file__), '../porcupine/resources/util/python'))

from porcupine import Porcupine
from util import *


# Waits for a wake word to be detected, once it is detected, records a short audio clip
# in a .wav file and sends it to the speech recognition server.
class WakeWordDetect:
  def __init__(self):
    rospy.init_node('vva_wake_word_detect')
    
    # Subscribed topics
    # ~ self.topic_sub =  rospy.Subscriber('topic', TYPE, self.topic_callback)
    
    # Published topics
    self.kinect_led_option_pub = rospy.Publisher('kinect_led_option', UInt16, queue_size=10)
    self.kinect_tilt_pub       = rospy.Publisher('kinect_tilt', Float64, queue_size=10)

    # Parameters
    self.rate                = rospy.get_param('~rate', 10)
    self._input_device_index = rospy.get_param('~input_device_index', 0)
    self._input_channels     = rospy.get_param('~input_channels', 1)
    self._sampling_rate      = rospy.get_param('~sampling_rate', 16000)
    self._mic_type_id        = rospy.get_param('~mic_type_id', "kinect")
    self._record_seconds     = rospy.get_param('~record_seconds', 5.0)
    self._record_file        = rospy.get_param('~record_file', os.path.join(os.path.dirname(__file__), '../records/') + "temp.wav")
    self._speech_recognition_service = rospy.get_param('~speech_recognition_service', "/vva_recognize_speech")
    self._enable_kinect_tilt = rospy.get_param('~enable_kinect_tilt', False)
    
    keyword_file_paths = rospy.get_param('~keyword_file_paths', "resources/keyword_files/raspberry-pi/blueberry_raspberry-pi.ppn")
    self._keyword_file_paths = [os.path.join(os.path.dirname(__file__), '../porcupine/') + x.strip() for x in keyword_file_paths.split(',')]
    
    sensitivities            = rospy.get_param('~sensitivities', "1.0")
    if isinstance(sensitivities, float):
      self._sensitivities = [sensitivities] * len(self._keyword_file_paths)
    else:
      self._sensitivities = [float(x) for x in sensitivities.split(',')]
    
    
    # Global variables for subscribed topics
    # ~ self.topic_name = None  
    
    # Other global variables
    self.porcupine = None
    self.pa = None
    self.porcupine_audio_stream = None
    self.keyword_names = list()
    self.num_keywords = len(self._keyword_file_paths)
    self.wav_audio_stream = None
    self.wav_audio_stream_frame_lenght = 1024

    

  # ==================================================
  # Read the subscribed topics
  # ==================================================
  
  # ~ def topic_callback(self,msg):
    # ~ self.topic_name = msg
  
    
  # ==================================================
  # Update function
  # ==================================================
  def update(self):
    pcm = self.porcupine_audio_stream.read(self.porcupine.frame_length, exception_on_overflow=False)
    pcm = struct.unpack_from("h" * self.porcupine.frame_length, pcm)

    result = self.porcupine.process(pcm)
    
    ## DEBUG
    # ~ rospy.loginfo("result: " + str(result))
    
    if self.num_keywords == 1 and result:
      rospy.loginfo('[%s] detected keyword' % str(datetime.now()))
      self.record_and_send_voice_command()
    elif self.num_keywords > 1 and result >= 0:
      rospy.loginfo('[%s] detected %s' % (str(datetime.now()), self.keyword_names[result]))
      self.record_and_send_voice_command()



  # ==================================================
  # Record voice command
  # ==================================================
  # The main loop is stopped during "record_seconds" while this function is executed,
  # hence no new wake words are detected during this time.
  def record_and_send_voice_command(self):

    if self._enable_kinect_tilt:
      # Ask Kinect to change the tilt to look up
      kinect_tilt_val = Float64()
      kinect_tilt_val.data = 20
      self.kinect_tilt_pub.publish(kinect_tilt_val)

    # Close the porcupine stream and open the wav stream
    self.porcupine_audio_stream.close()

    self.wav_audio_stream = self.pa.open(
                     rate=self._sampling_rate,
                     channels=self._input_channels,
                     format=pyaudio.paInt16,
                     input=True,
                     frames_per_buffer=self.wav_audio_stream_frame_lenght,
                     input_device_index=self._input_device_index)

    if self._enable_kinect_tilt:
      # Wait while the Kinect changes the tilt
      rospy.sleep(1.0)
    
    # Change the color of the LED of the Kinect to orange (3) to indicate that is listening
    kinect_led_val = UInt16()
    kinect_led_val.data = 3
    self.kinect_led_option_pub.publish(kinect_led_val)

    rospy.loginfo("Recording for %.1f seconds...", self._record_seconds)
    frames = []
    for i in range(0, int(self._sampling_rate / self.wav_audio_stream_frame_lenght * self._record_seconds)):
        data = self.wav_audio_stream.read(self.wav_audio_stream_frame_lenght, exception_on_overflow=False)
        frames.append(data)
    rospy.loginfo("Finished recording")
    
    # Turn off the LED of the Kinect to indicate that it is sending the file to the server.
    kinect_led_val.data = 0
    self.kinect_led_option_pub.publish(kinect_led_val)

    # Save the record in a .wav file 
    waveFile = wave.open(self._record_file, 'wb')
    waveFile.setnchannels(self._input_channels)
    waveFile.setsampwidth(self.pa.get_sample_size(pyaudio.paInt16))
    waveFile.setframerate(self._sampling_rate)
    waveFile.writeframes(b''.join(frames))
    waveFile.close()
  
    # Send the .wav file to the speech recognition server
    status = ""
    try:
      rospy.loginfo("Sending voice recording to the Speech Recognition node...")
      srv_recognize_speech = rospy.ServiceProxy(self._speech_recognition_service, VVARecognizeSpeech)
      with open(self._record_file, "rb") as bin_file:
        response = srv_recognize_speech(self._mic_type_id, bin_file.read())
      status = response.status
      rospy.loginfo('Speech Recognition result: "%s". Text: "%s"', status, response.text)
    except rospy.ServiceException as e:
      rospy.logwarn("Service failed: " + str(e))
      status = "ERROR"

    # Change the color of the LED of the Kinect according to the server's reply.
    if status.startswith('SUCCESS'):
      kinect_led_val.data = 1  # Green hold
    else:
      kinect_led_val.data = 2  # Red
    self.kinect_led_option_pub.publish(kinect_led_val)

    if self._enable_kinect_tilt:
      # Change back the tilt of the Kinect to 0 degrees
      kinect_tilt_val.data = 0
      self.kinect_tilt_pub.publish(kinect_tilt_val)

    # Close the wav stream and open again the porcupine stream
    self.wav_audio_stream.close()
    
    self.porcupine_audio_stream = self.pa.open(
                 rate=self.porcupine.sample_rate,
                 channels=1,
                 format=pyaudio.paInt16,
                 input=True,
                 frames_per_buffer=self.porcupine.frame_length,
                 input_device_index=self._input_device_index)
    
    # Wait to let the user see the status in the LED of the Kinect
    rospy.sleep(3)

    # Change back the color of the LED of the Kinect to blinking green (4) to indicate that Speech Recognition has finished.
    kinect_led_val.data = 4
    self.kinect_led_option_pub.publish(kinect_led_val)


  # ==================================================
  # Main loop
  # ==================================================
  def spin(self):
    rospy.loginfo("Start vva_wake_word_detect")
    rate = rospy.Rate(self.rate)
    rospy.on_shutdown(self.shutdown)
    
    rospy.loginfo("Waiting for %s service to be started...", self._speech_recognition_service)
    rospy.wait_for_service(self._speech_recognition_service)
    rospy.loginfo("%s service started.", self._speech_recognition_service)
    
    for x in self._keyword_file_paths:
      self.keyword_names.append(os.path.basename(x).replace('.ppn', '').replace('_compressed', '').split('_')[0])

    rospy.loginfo('listening for:')
    for keyword_name, sensitivity in zip(self.keyword_names, self._sensitivities):
      rospy.loginfo('- %s (sensitivity: %.2f)' % (keyword_name, sensitivity))

    self.porcupine = Porcupine(
                  library_path=LIBRARY_PATH,     # defined in porcupine/resources/util/python/util.py
                  model_file_path=MODEL_FILE_PATH,   # defined in porcupine/resources/util/python/util.py
                  keyword_file_paths=self._keyword_file_paths,
                  sensitivities=self._sensitivities)

    ## DEBUG
    # ~ rospy.loginfo('porcupine_audio_stream frames_per_buffer: ' + str(self.porcupine.frame_length) + ', rate: ' + str(self.porcupine.sample_rate))

    self.pa = pyaudio.PyAudio()
    self.porcupine_audio_stream = self.pa.open(
                     rate=self.porcupine.sample_rate,
                     channels=1,
                     format=pyaudio.paInt16,
                     input=True,
                     frames_per_buffer=self.porcupine.frame_length,
                     input_device_index=self._input_device_index)

    while not rospy.is_shutdown():
      self.update()
      rate.sleep()
    rospy.spin()


  # ==================================================
  # If shutdown
  # ==================================================

  def shutdown(self):
    rospy.loginfo("Stop vva_wake_word_detect. CHAO PESCAO!")
    
    if self.porcupine is not None:
      self.porcupine.delete()

    if self.porcupine_audio_stream is not None:
      self.porcupine_audio_stream.close()
      
    if self.wav_audio_stream is not None:
      self.wav_audio_stream.close()

    if self.pa is not None:
      self.pa.terminate()
    
    # Stop message
    rospy.sleep(1)
    

# ==================================================
# Main
# ==================================================
def main():
  wake_word_detect = WakeWordDetect()
  wake_word_detect.spin()

if __name__ == '__main__':
  main()









