#!/usr/bin/env python

#------------------------------------------------------------------------------
# Copyright (c) 2020, Andres A. <andres.arboleda AT gmail DOT com>
# All rights reserved.
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
#------------------------------------------------------------------------------

from threading import Thread
import socket
import audioTranscript
import wavTranscriber
import logging
import argparse
import sys
import os


# Debug helpers
logging.basicConfig(stream=sys.stderr, level=logging.INFO)


class VVASocketServer(Thread):
  """
  VVA Socket Server class.
  """
  def __init__(self, args):
    """
    Constructor.
    """
    Thread.__init__(self)
    
    # Parse all the arguments
    parser = argparse.ArgumentParser(description='Transcribe audio files using webRTC VAD after receiving a notification via TCP socket')
    parser.add_argument('--aggressive', type=int, choices=range(4), required=False,
                        help='Determines how aggressive filtering out non-speech is. (Interger between 0-3)')
    parser.add_argument('--audio', required=True,
                        help='Path to the audio file to run (WAV format)')
    parser.add_argument('--model', required=True,
                        help='Path to directory that contains all model files (output_graph and scorer)')
    parser.add_argument('--port', required=True,
                        help='TCP port used to receive the notifications of a new file from the ROS node. It listens on localhost')
    args = parser.parse_args()
    if args.audio is None or args.model is None or args.port is None:
        parser.print_help()
        parser.exit()
        
    self.audio_file = args.audio
    if args.aggressive is not None:
      self.aggressive = args.aggressive
    else:
      self.aggressive = 1

    # Point to a path containing the pre-trained models & resolve ~ if used
    dirName = os.path.expanduser(args.model)

    # Resolve all the paths of model files
    output_graph, scorer = wavTranscriber.resolve_models(dirName)

    # Load output_graph, alpahbet and scorer
    self.model_retval = wavTranscriber.load_model(output_graph, scorer)


    # Create the server TCP socket  
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.sock.bind(('localhost', int(args.port)))
    self.sock.listen(1)
    
    # Start the socket connections attending thread
    self.start()


  def run(self):
    """
    Threaded function that processes each TCP connection.
    """
    while True:
        logging.info("Waiting for TCP socket connection...")
        # accept connections from outside
        (clientsocket, client_address) = self.sock.accept()
        logging.info("TCP socket connection received")
        
        message = clientsocket.recv(16) # Max. 16 bytes = 16 characters
   
        logging.debug("Socket message received: " + str(message))
        
        if message == b'exit':
          break
        
        logging.debug("Starting audio transcription...")
        # Perform the audio transcript
        transcription = audioTranscript.transcribeAudio(self.audio_file, self.aggressive, self.model_retval)
        
        clientsocket.send(bytes(transcription, 'utf-8'))
        clientsocket.close()
      
    logging.info("Exit request received. Closing sockets and finishing server thread...")
    clientsocket.close()
    self.sock.close()


if __name__ == '__main__':
  vva_socket_server = VVASocketServer(sys.argv[1:])




