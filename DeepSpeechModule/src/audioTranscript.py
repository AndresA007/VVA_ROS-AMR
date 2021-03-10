#!/usr/bin/env python

#------------------------------------------------------------------------------
# Copyright (c) 2020, mozilla.org
# Copyright (c) 2020, Andres A. <andres.arboleda AT gmail DOT com>
# All rights reserved.
#
# License: Mozilla Public License Version 2.0
# License copy in: http://mozilla.org/MPL/2.0/
# 
# Based on the code of "vad_transcriber" published by:
# mozilla.org
# 
# Github repo of original code:
# https://github.com/mozilla/DeepSpeech-examples/tree/r0.8/vad_transcriber
# 
# This version modified by:
# andres.arboleda AT gmail DOT com, (sep/2020)
#------------------------------------------------------------------------------

import os
import logging
import numpy as np
import wavTranscriber
from pydub import AudioSegment, effects


def transcribeAudio(waveFile, aggressiveness, model_retval):


    inference_time = 0.0

    # Run VAD on the input file
    segments, sample_rate, audio_length, num_channels = wavTranscriber.vad_segment_generator(waveFile, aggressiveness)

    output = []
    concat_transcripts = ""
    # Run deepspeech on each channel separately
    for channel in range(num_channels):
      logging.debug("Processing channel: %d:" % (channel,))
      
      channel_output = []
      for i, segment in enumerate(segments[channel]):
          # Run deepspeech on the chunk that just completed VAD
          logging.debug("Processing chunk %002d" % (i,))
          
          # Normalize the volume
          pydub_audio_segment = AudioSegment(data=segment, sample_width=2, frame_rate=sample_rate, channels=1)
          normalizedsound = effects.normalize(pydub_audio_segment)

          audio = np.frombuffer(normalizedsound.raw_data, dtype=np.int16)
          channel_output.append(wavTranscriber.stt(model_retval[0], audio, sample_rate))
          
          
          inference_time += channel_output[i][1]
          logging.debug('Transcript: "%s"', channel_output[i][0])
          # Concatenate the transcripts of all the channels and all the chunks
          concat_transcripts += channel_output[i][0] + "; "
          
      output.append(channel_output)
      ## DEBUG
      logging.info("Transcript output per channel and per chunk: " + str(output))

    # Extract filename from the full file path
    filename, ext = os.path.split(os.path.basename(waveFile))
    title_names = ['Filename', 'Duration(s)', 'Inference Time(s)', 'Model Load Time(s)', 'Scorer Load Time(s)']
    logging.info("\n%-30s %-20s %-20s %-20s %s" % (title_names[0], title_names[1], title_names[2], title_names[3], title_names[4]))
    logging.info("%-30s %-20.3f %-20.3f %-20.3f %-0.3f" % (filename + ext, audio_length, inference_time, model_retval[1], model_retval[2]))
    
    return concat_transcripts


