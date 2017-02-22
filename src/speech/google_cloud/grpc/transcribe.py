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
"""Sample that transcribes a FLAC audio file stored in Google Cloud Storage,
using GRPC."""

import argparse

import google.auth
import google.auth.transport.grpc
import google.auth.transport.requests
from google.cloud.grpc.speech.v1beta1 import cloud_speech_pb2
import base64
import time
from google.longrunning import operations_pb2
from google.cloud.grpc.speech.v1beta1.cloud_speech_pb2 import SpeechContext

# Keep the request alive for this many seconds
DEADLINE_SECS = 60
SPEECH_SCOPE = 'https://www.googleapis.com/auth/cloud-platform'


def make_channel(host, port):
    """Creates a secure channel with auth credentials from the environment."""
    # Grab application default credentials from the environment
    credentials, _ = google.auth.default(scopes=[SPEECH_SCOPE])

    # Create a secure channel using the credentials.
    http_request = google.auth.transport.requests.Request()
    target = '{}:{}'.format(host, port)

    return google.auth.transport.grpc.secure_authorized_channel(
        credentials, http_request, target)


def main(raw_file_path, encoding, sample_rate, language_code='en-US'):
    channel = make_channel('speech.googleapis.com', 443)
    service = cloud_speech_pb2.SpeechStub(channel)

    speech_file = open(raw_file_path, 'rb')

    speech_content = speech_file.read()#base64.b64encode(speech_file.read())

    # The method and parameters can be inferred from the proto from which the
    # grpc client lib was generated. See:
    # https://github.com/googleapis/googleapis/blob/master/google/cloud/speech/v1beta1/cloud_speech.proto
    operation = service.AsyncRecognize(cloud_speech_pb2.AsyncRecognizeRequest(
        config=cloud_speech_pb2.RecognitionConfig(
            # There are a bunch of config options you can specify. See
            # https://goo.gl/KPZn97 for the full list.
            encoding=encoding,  # one of LINEAR16, FLAC, MULAW, AMR, AMR_WB
            sample_rate=sample_rate,  # the rate in hertz
            # See https://g.co/cloud/speech/docs/languages for a list of
            # supported languages.
            language_code=language_code,  # a BCP-47 language tag
            max_alternatives = 20,
            speech_context = SpeechContext(phrases=['shiqi']),
        ),
        audio=cloud_speech_pb2.RecognitionAudio(
            content=speech_content
        )
    ), DEADLINE_SECS)

    # Construct a long running operation endpoint.
    service = operations_pb2.OperationsStub(channel)

    name = operation.name

    while True:
        # Give the server a few seconds to process.
        print('Waiting for server processing...')
        time.sleep(1)
        operation = service.GetOperation(
            operations_pb2.GetOperationRequest(name=name),
            DEADLINE_SECS)

        if operation.error.message:
            print('\nOperation error:\n{}'.format(operation.error))

        if operation.done:
            break

    response = cloud_speech_pb2.AsyncRecognizeResponse()
    operation.response.Unpack(response)
    # Print the recognition result alternatives and confidence scores.
    for result in response.results:
        print('Result:')
        for alternative in result.alternatives:
            print(u'  ({}): {}'.format(
                alternative.confidence, alternative.transcript))



def _gcs_uri(text):
    if not text.startswith('gs://'):
        raise ValueError(
            'Cloud Storage uri must be of the form gs://bucket/path/')
    return text


PROTO_URL = ('https://github.com/googleapis/googleapis/blob/master/'
             'google/cloud/speech/v1beta1/cloud_speech.proto')
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('raw_file_path', type=str)
    parser.add_argument(
        '--encoding', default='LINEAR16', choices=[
            'LINEAR16', 'FLAC', 'MULAW', 'AMR', 'AMR_WB'],
        help='How the audio file is encoded. See {}#L67'.format(PROTO_URL))
    parser.add_argument('--sample_rate', type=int, default=16000)

    args = parser.parse_args()
    main(args.raw_file_path, args.encoding, args.sample_rate)
