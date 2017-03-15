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

        #Monitor progress of what we've been able to run recognition on. 
        self.open_log_files()

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

    def open_log_files(self): 
        #Read in successfully recognized files. 
        successes_file = open('successes.txt', 'r')
        self.successes = [line.strip() for line in successes_file]
        successes_file.close()

        #Now read in failures. 
        failures_file = open('failures.txt', 'r')
        self.failures = [line.strip() for line in failures_file]
        failures_file.close()

        #Now open the files in append mode to keep adding to them. 
        self.successes_file = open('successes.txt', 'a')
        self.failures_file = open('failures.txt', 'a')

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
                #speech_context = SpeechContext(phrases=self.speech_context),
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

            try: 
                out_file = open(output_file, 'w')    

                for result in response.results: 
                    for alternative in result.alternatives:
                        out_file.write(alternative.transcript + ';' + str(alternative.confidence) + '\n')
         
                out_file.close()

                #Keep track of success. 
                self.successes_file.write(output_file + '\n')

            except Exception: 
                #Keep track of failure. 
                self.failures_file.write(output_file + '\n')


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

                #If we've already processed this file, then skip it. 
                if not out_file in self.successes and not out_file in self.failures:
                    #Now run recognition request on the file. 
                    self.recognize(rec_path, output_file = out_file)

                print 'Ran recognition on file: ' + file_name

    def recognize_from_file(self, in_file_name, in_folder_path, out_folder_path): 
        """
        Will create n-best results files for each
        audio file specified within a given text file. 
        Will write results to individual files within the
        output folder.
        """
        in_file = open(in_file_name, 'r')

        #Get file name and add raw extension in case files come with other extension.  
        file_names = [os.path.splitext(os.path.basename(line).strip())[0] for line in in_file]

        #Get pertaining path and file names for all possible recordings to retrieve. 
        recordings = []

        #Get file name and path pairs in the recordings folder. 
        for usr_folder in os.listdir(in_folder_path): 
            #Path where recordings are kept for this user. 
            usr_path = in_folder_path + usr_folder + '/recordings/'

            for recording in os.listdir(usr_path):
                #Path to recording. 
                rec_path = usr_path + recording

                #Recording file name. 
                rec_name = os.path.basename(os.path.splitext(rec_path)[0])

                recordings.append([rec_name, rec_path])

        #Now go through recordings and pull out path for the ones we need to run recognition on. 
        for rec_name, rec_path in recordings: 
            #True if this is a file we want to recognize. 
            if rec_name in file_names:
                out_file_name = out_folder_path + rec_name + '.nbest'

                #Attempt to recognize speech in file.             
                self.recognize(rec_path, output_file=out_file_name)

                print 'Ran recognition on file: ' + rec_name


    def verify_recognition(self, in_folder_path, out_folder_path): 
        """
        Verifies that there is a recognition file in the
        output folder for every audio file in the input folder. 
        """

        #Get input audio file names. 
        in_names = set()

        for usr_folder in os.listdir(in_folder_path):
            #Path where recordings are kept for this user. 
            usr_path = in_folder_path + usr_folder + '/recordings/'

            for recording in os.listdir(usr_path):
                #Recording file name. 
                rec_name = os.path.splitext(recording)[0]

                in_names.add(rec_name)

        #Get output file names
        out_names = set()

        for output_file in os.listdir(out_folder_path):
            file_name = os.path.splitext(output_file)[0]

            out_names.add(file_name)

        #Check for files in which sets are disjoint. 
        disjoint = in_names.symmetric_difference(out_names)

        #Make sure they match. 
        if not len(disjoint) == 0: 
            print 'File sets do not match!!'
            print disjoint
        else:
            print 'File sets match!'
            
    
    def _gcs_uri(self, text):
        if not text.startswith('gs://'):
            raise ValueError(
                'Cloud Storage uri must be of the form gs://bucket/path/')
        return text


if __name__ == '__main__':
    google_speech = GoogleSpeech()

    if sys.argv[1] == 'recognize_folder': 
        google_speech.recognize_folder(sys.argv[2], sys.argv[3])
    elif sys.argv[1] == 'recognize_from_file':
        google_speech.recognize_from_file(sys.argv[2], sys.argv[3], sys.argv[4])
    elif sys.argv[1] == 'verify': 
        google_speech.verify_recognition(sys.argv[2], sys.argv[3])
