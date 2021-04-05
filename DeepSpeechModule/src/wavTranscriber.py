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

import glob
import webrtcvad
import logging
import wavSplit
from deepspeech import Model
from timeit import default_timer as timer
import numpy as np

'''
Load the pre-trained model into the memory
@param models: Output Grapgh Protocol Buffer file
@param scorer: Scorer file

@Retval
Returns a list [DeepSpeech Object, Model Load Time, Scorer Load Time]
'''
def load_model(models, scorer):
    model_load_start = timer()
    ds = Model(models)
    model_load_end = timer() - model_load_start
    logging.debug("Loaded model in %0.3fs." % (model_load_end))

    scorer_load_start = timer()
    ds.enableExternalScorer(scorer)
    scorer_load_end = timer() - scorer_load_start
    logging.debug('Loaded external scorer in %0.3fs.' % (scorer_load_end))

    return [ds, model_load_end, scorer_load_end]

'''
Run Inference on input audio file
@param ds: Deepspeech object
@param audio: Input audio for running inference on
@param fs: Sample rate of the input audio file

@Retval:
Returns a list [Inference, Inference Time, Audio Length]

'''
def stt(ds, audio, fs):
    inference_time = 0.0
    audio_length = len(audio) * (1 / fs)

    # Run Deepspeech
    logging.debug('Running inference...')
    inference_start = timer()
    output = ds.stt(audio)
    inference_end = timer() - inference_start
    inference_time += inference_end
    logging.debug('Inference took %0.3fs for %0.3fs audio file.' % (inference_end, audio_length))

    return [output, inference_time]

'''
Resolve directory path for the models and fetch each of them.
@param dirName: Path to the directory containing pre-trained models

@Retval:
Retunns a tuple containing each of the model files (pb, scorer)
'''
def resolve_models(dirName):
    pb = glob.glob(dirName + "/*.tflite")[0]
    logging.debug("Found Model: %s" % pb)

    scorer = glob.glob(dirName + "/*.scorer")[0]
    logging.debug("Found scorer: %s" % scorer)

    return pb, scorer

'''
Generate VAD segments. Filters out non-voiced audio frames.
@param waveFile: Input wav file to run VAD on.0

@Retval:
Returns tuple of
    segments: a bytearray of multiple smaller audio frames
              (The longer audio split into mutiple smaller one's)
    sample_rate: Sample rate of the input audio file
    audio_length: Duraton of the input audio file

'''
def vad_segment_generator(wavFile, aggressiveness):
    logging.debug("Caught the wav file @: %s" % (wavFile))
    audio, sample_rate, audio_length, num_channels = wavSplit.read_wave(wavFile)
    assert sample_rate == 16000, "Only 16000Hz input WAV files are supported for now!"
    vad = webrtcvad.Vad(int(aggressiveness))

    # Separate the audio channels
    audio_channel_data = []
    segments = []
    audio_np_data = np.fromstring(audio, dtype=np.int16)
    for i in range(num_channels):
      # Extract the data of only one channel
      audio_channel_data.append(audio_np_data[i::num_channels])
      audio_channel = audio_channel_data[i].tostring()
      frames = wavSplit.frame_generator(30, audio_channel, sample_rate)
      frames = list(frames)
      segments.append(wavSplit.vad_collector(sample_rate, 30, 300, vad, frames))
      i += 1

    return segments, sample_rate, audio_length, num_channels



