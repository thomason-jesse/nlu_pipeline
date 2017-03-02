#!/usr/bin/python
# Copyright (C) 2016 Google Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#            http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import sys
import google.auth
import google.auth.transport.grpc
import google.auth.transport.requests
from google.cloud.grpc.speech.v1beta1 import cloud_speech_pb2
from google.cloud.grpc.speech.v1beta1.cloud_speech_pb2 import SpeechContext
import os
import ntpath

# Keep the request alive for this many seconds
DEADLINE_SECS = 60
SPEECH_SCOPE = 'https://www.googleapis.com/auth/cloud-platform'

PROTO_URL = ('https://github.com/googleapis/googleapis/blob/master/'
             'google/cloud/speech/v1beta1/cloud_speech.proto')

class GoogleSpeech:
    def __init__(self): 
        self.service = cloud_speech_pb2.SpeechStub(
            self.make_channel('speech.googleapis.com', 443))

        self.speech_context = ['stacy', "stacy's", 'miller', "miller's",
                               'scott', "scott's", 'niekum', "niekum's",
                               'jesse', "jesse's", 'thomason', "thomason's", 
                               'shiqi', "shiqi's", 'zhang', "zhang's",
                               'jivko', "jivko's", 'sinapov', "sinapov's",
                               'rodolfo', "rodolfo's", 'corona', "corona's",
                               'aishwarya', "aishwarya's", 'padmakumar', "padmakumar's",
                               'peter', "peter's", 'stone', "stone's",
                               'dana', "dana's", 'ballard', "ballard's", 
                               'ray', "ray's", 'mooney', "mooney's", 
                               'bruce', "bruce's", 'porter', "porter's"]


    def make_channel(self, host, port):
        """Creates a secure channel with auth credentials from the environment."""
        # Grab application default credentials from the environment
        credentials, _ = google.auth.default(scopes=[SPEECH_SCOPE])

        # Create a secure channel using the credentials.
        http_request = google.auth.transport.requests.Request()
        target = '{}:{}'.format(host, port)

        return google.auth.transport.grpc.secure_authorized_channel(
            credentials, http_request, target)

    def recognize(self, input_flac, encoding='FLAC', sample_rate=16000, language_code='en-US', output_file=None):
        #Read in audio data. 
        speech_content = open(input_flac, 'r').read()

        # The method and parameters can be inferred from the proto from which the
        # grpc client lib was generated. See:
        # https://github.com/googleapis/googleapis/blob/master/google/cloud/speech/v1beta1/cloud_speech.proto
        response = self.service.SyncRecognize(cloud_speech_pb2.SyncRecognizeRequest(
            config=cloud_speech_pb2.RecognitionConfig(
                # There are a bunch of config options you can specify. See
                # https://goo.gl/KPZn97 for the full list.
                encoding=encoding,  # one of LINEAR16, FLAC, MULAW, AMR, AMR_WB
                sample_rate=sample_rate,  # the rate in hertz
                # See https://g.co/cloud/speech/docs/languages for a list of
                # supported languages.
                language_code=language_code,  # a BCP-47 language tag
                speech_context = SpeechContext(phrases=self.speech_context),
                max_alternatives=20,
            ),
            audio=cloud_speech_pb2.RecognitionAudio(
                content=speech_content
            )
        ), DEADLINE_SECS)

        # Print the recognition result alternatives and confidence scores.
        if output_file == None:
            for result in response.results:
                print('Result:')
                for alternative in result.alternatives:
                    print(u'  ({}): {}'.format(
                        alternative.confidence, alternative.transcript))
        #Write results to file. 
        else: 
            out_file = open(output_file, 'w')    

            for result in response.results: 
                for alternative in result.alternatives:
                    out_file.write(alternative.transcript + ';' + str(alternative.confidence) + '\n')

            out_file.close()

    def recognize_folder(self, in_folder_path, out_folder_path):
        """
        Will create n-best result files for each 
        file in a given corpus folder. Will write
        results to individual files within the
        given output folder. 
        """
        for usr_folder in os.listdir(in_folder_path):
            #Path where recordings are kept for this user. 
            usr_path = in_folder_path + usr_folder + '/recordings/'

            for recording in os.listdir(usr_path):
                #Path to recording. 
                rec_path = usr_path + recording
                
                #Path to output file. 
                file_name = ntpath.basename(rec_path).split('.')[0]
                out_file = out_folder_path + file_name + '.nbest'

                #Now run recognition request on the file. 
                self.recognize(rec_path, output_file = out_file)

                print 'Ran recognition on file: ' + file_name

    def _gcs_uri(self, text):
        if not text.startswith('gs://'):
            raise ValueError(
                'Cloud Storage uri must be of the form gs://bucket/path/')
        return text


if __name__ == '__main__':
    google_speech = GoogleSpeech()

    google_speech.recognize_folder(sys.argv[1], sys.argv[2])
