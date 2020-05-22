#!/usr/bin/env python

# Copyright 2019 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#	  http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Google Cloud Speech API sample application using the streaming API.
NOTE: This module requires the dependencies `pyaudio` and `termcolor`.
To install using pip:
	pip install pyaudio
	pip install termcolor
Example usage:
	python transcribe_streaming_infinite.py
"""

# [START speech_transcribe_infinite_streaming]

import time
import re
import sys
import threading
import json

# uses result_end_time currently only avaialble in v1p1beta, will be in v1 soon
import google
from google.cloud import speech_v1p1beta1 as speech
import pyaudio
from six.moves import queue
from google.api_core.retry import Retry
from google.oauth2 import service_account

# Audio recording parameters
STREAMING_LIMIT = 10000
SAMPLE_RATE = 44100
CHUNK_SIZE = 1024#int(SAMPLE_RATE / 10)  # 100ms

RED = '\033[0;31m'
GREEN = '\033[0;32m'
YELLOW = '\033[0;33m'


def get_current_time():
	"""Return Current Time in MS."""

	return int(round(time.time() * 1000))


class ResumableMicrophoneStream:
	"""Opens a recording stream as a generator yielding the audio chunks."""

	def __init__(self, rate, chunk_size):
		self._rate = rate
		self.chunk_size = chunk_size
		self._num_channels = 1
		self._buff = queue.Queue()
		self.closed = True
		self.start_time = get_current_time()
		self.restart_counter = 0
		self.audio_input = []
		self.last_audio_input = []
		self.result_end_time = 0
		self.is_final_end_time = 0
		self.final_request_end_time = 0
		self.bridging_offset = 0
		self.last_transcript_was_final = False
		self.new_stream = True
		self._audio_interface = pyaudio.PyAudio()
		self._audio_stream = self._audio_interface.open(
			format=pyaudio.paInt16,
			channels=self._num_channels,
			rate=self._rate,
			input=True,
			frames_per_buffer=self.chunk_size,
			# Run the audio stream asynchronously to fill the buffer object.
			# This is necessary so that the input device's buffer doesn't
			# overflow while the calling thread makes network requests, etc.
			stream_callback=self._fill_buffer,
		)

	def __enter__(self):

		self.closed = False
		return self

	def __exit__(self, type, value, traceback):

		self._audio_stream.stop_stream()
		self._audio_stream.close()
		self.closed = True
		# Signal the generator to terminate so that the client's
		# streaming_recognize method will not block the process termination.
		self._buff.put(None)
		self._audio_interface.terminate()

	def close(self):
		self._audio_stream.stop_stream()
		self._audio_stream.close()
		self.closed = True
		# Signal the generator to terminate so that the client's
		# streaming_recognize method will not block the process termination.
		self._buff.put(None)
		self._audio_interface.terminate()

	def _fill_buffer(self, in_data, *args, **kwargs):
		"""Continuously collect data from the audio stream, into the buffer."""

		self._buff.put(in_data)
		return None, pyaudio.paContinue

	def generator(self):
		"""Stream Audio from microphone to API and to local buffer"""

		while not self.closed:
			data = []

			if self.new_stream and self.last_audio_input:

				chunk_time = STREAMING_LIMIT / len(self.last_audio_input)

				if chunk_time != 0:

					if self.bridging_offset < 0:
						self.bridging_offset = 0

					if self.bridging_offset > self.final_request_end_time:
						self.bridging_offset = self.final_request_end_time

					chunks_from_ms = round((self.final_request_end_time -
											self.bridging_offset) / chunk_time)

					self.bridging_offset = (round((
						len(self.last_audio_input) - chunks_from_ms)
												  * chunk_time))

					for i in range(chunks_from_ms, len(self.last_audio_input)):
						data.append(self.last_audio_input[i])

				self.new_stream = False

			# Use a blocking get() to ensure there's at least one chunk of
			# data, and stop iteration if the chunk is None, indicating the
			# end of the audio stream.
			chunk = self._buff.get()
			self.audio_input.append(chunk)

			if chunk is None:
				return
			data.append(chunk)
			# Now consume whatever other data's still buffered.
			while True:
				try:
					chunk = self._buff.get(block=False)

					if chunk is None:
						return
					data.append(chunk)
					self.audio_input.append(chunk)

				except queue.Empty:
					break

			yield b''.join(data)

class SingleStreamSTT:

	def __init__(self):
		self.timer_counter = [-1]
		self.timer_lock = threading.Lock()
		self.record = False

	def listen_print_loop(self, responses, stream, received_callback, send_in_progress_text_callback, activation_time):
		"""Iterates through server responses and prints them.
		The responses passed is a generator that will block until a response
		is provided by the server.
		Each response may contain multiple results, and each result may contain
		multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
		print only the transcription for the top alternative of the top result.
		In this case, responses are provided for interim results as well. If the
		response is an interim one, print a line feed at the end of it, to allow
		the next result to overwrite it, until the response is a final one. For the
		final one, print a newline to preserve the finalized transcription.
		"""

		for response in responses:

			if get_current_time() - stream.start_time > STREAMING_LIMIT:
				stream.start_time = get_current_time()
				break

			if not response.results:
				continue

			result = response.results[0]

			if not result.alternatives:
				continue

			transcript = result.alternatives[0].transcript

			result_seconds = 0
			result_nanos = 0

			# Display interim results, but with a carriage return at the end of the
			# line, so subsequent lines will overwrite them.
			if result.is_final:
				self.timer_counter[0] = 2
				sys.stdout.write(GREEN)
				sys.stdout.write('\033[K')
				sys.stdout.write(transcript + '\n')
					#process_utterance(transcript, recorded_time_passed)

				# Exit recognition if any of the transcribed phrases could be
				# one of our keywords.
				# COMMENT BACK IN TO ALLOW VERBAL EXITING
				'''
				if re.search(r'\b(exit|quit)\b', transcript, re.I):
					sys.stdout.write(YELLOW)
					sys.stdout.write('Exiting...\n')
					stream.closed = True
					break
				'''

				received_callback(transcript, activation_time)

				# we are done here
				break

			else:
				self.timer_counter[0] = 2
				sys.stdout.write(RED)
				sys.stdout.write('\033[K')
				sys.stdout.write(transcript + '\r')
				send_in_progress_text_callback(transcript)

				stream.last_transcript_was_final = False

				#check_pre_utterance(transcript)

	def run_terminal(self, receival_callback, send_in_progress_text_callback, receive_agent_highlight, activation_time, stream_lock):
		"""start bidirectional streaming from microphone input to speech API"""

		if stream_lock.acquire(blocking=False):

			try:
				dialogflow_key = json.load(open('speech_key.json'))
				credentials = (service_account.Credentials.from_service_account_info(dialogflow_key))
				client = speech.SpeechClient(credentials=credentials)
			except:
				print("could not find or load google API service key")
				print("returning without calling STT")
				stream_lock.release()
				return

			config = speech.types.RecognitionConfig(
				encoding=speech.enums.RecognitionConfig.AudioEncoding.LINEAR16,
				sample_rate_hertz=SAMPLE_RATE,
				language_code='en-US',
				max_alternatives=1,
				enable_automatic_punctuation=False,
				enable_word_time_offsets=True,)
			streaming_config = speech.types.StreamingRecognitionConfig(
				config=config,
				interim_results=True)
				#single_utterance=True)

			mic_manager = ResumableMicrophoneStream(SAMPLE_RATE, CHUNK_SIZE)
			print(mic_manager.chunk_size)
			sys.stdout.write(YELLOW)
			sys.stdout.write('\nListening, say "Quit" or "Exit" to stop.\n\n')
			sys.stdout.write('End (ms)	   Transcript Results/Status\n')
			sys.stdout.write('=====================================================\n')

			receive_agent_highlight()

			self.timer_lock.acquire()
			with mic_manager as stream:

				sys.stdout.write(YELLOW)
				sys.stdout.write('\n' + str(
					STREAMING_LIMIT * stream.restart_counter) + ': NEW REQUEST\n')

				stream.audio_input = []
				audio_generator = stream.generator()

				requests = (speech.types.StreamingRecognizeRequest(
					audio_content=content)for content in audio_generator)

				try:
					responses = client.streaming_recognize(streaming_config,
																   requests)#,retry=retry)

					self.timer_counter[0] = 2
					thread = threading.Thread(target=self.timeout_timer, args=(self.timer_callback,responses))
					thread.daemon = True
					thread.start()

					# Now, put the transcription responses to use.
					self.listen_print_loop(responses, stream, receival_callback, send_in_progress_text_callback, activation_time)
				except google.api_core.exceptions.DeadlineExceeded:
					print("Deadline exceeded. Closing stream.")
				except google.api_core.exceptions.Cancelled:
					print("Cancelled by application")

			print("stream thread ending")
			receive_agent_highlight()

			self.timer_lock.release()
			stream_lock.release()
		else:
			print("could not acquire stream lock")

	def timer_callback(self, stream):
		'''
		Callback to close the stream prematurely
		'''
		stream.close()

	def timeout_timer(self, callback, responses):
		while True:
			if self.timer_lock.acquire(blocking=False):
				self.timer_lock.release()
				break
			print(self.timer_counter[0])
			time.sleep(1)
			if self.timer_counter[0] > -1:
				self.timer_counter[0] -= 1
				if self.timer_counter[0] == -1:
					responses.cancel()
					break