# mycroft
import sys
import time
import os
# import mycroft path
mycroft_path = sys.argv[1]
sys.path.append(mycroft_path)
sys.path.append("{}/runner".format(mycroft_path))
# import folder containing stt source code
from precise.util import activate_notify
from precise_runner import PreciseRunner, PreciseEngine

# google real-time STT
from commanded_stt.single_stream_stt import *

# ros messages
from std_msgs.msg import String as RosString
from std_msgs.msg import Bool, Empty
from wisc_ros2_msgs.msg import StringArray
from wisc_ros2_msgs.msg import ActivationState

import rclpy
from rclpy.node import Node

class Controller(Node):

	def __init__(self):
		super().__init__('listener_node')

		# setup publishers and subscribers
		self.command_triggered_pub = self.create_publisher(RosString,'/stt/triggered',10)
		self.in_progress_stt_pub = self.create_publisher(StringArray,'/stt/in_progress',10)
		self.finished_stt_pub = self.create_publisher(StringArray,'/stt/result',10)

		self.lock_speech_sub = self.create_subscription(Empty, '/ctrl/lock_stt', self.lock_speech, 10)  # nothing but an unlock command can trigger the speech
		self.unlock_speech_sub = self.create_subscription(Empty, '/ctrl/unlock_stt', self.unlock_speech, 10)

		self.external_trigger_sub = self.create_subscription(RosString,'/stt/trigger',self.stt_triggered_callback,10)
		self.set_listeners_sub = self.create_subscription(ActivationState, "/stt/set_active", self.set_active_listeners, 10)
		self.set_perpetual_listener_sub = self.create_subscription(Bool, "/stt/perpetual", self.set_perpetual_stt_cb, 10)

		# setup modified google real-time stt
		self.single_stream_stt = SingleStreamSTT()

		# semaphore for the stream callback
		self.stream_lock = threading.Lock()
		self.speech_locked = False

		# countdown
		self.perpetual_listener = False
		self.countdown = -1
		self.countdown_lock = threading.Lock()
		self.countdown_method_lock = threading.Lock()

		# store active engines
		self.engines = {}
		for item in os.listdir("install/commanded_stt/share/commanded_stt"):
			if len(item) > 3 and item[-3:] == ".pb":
				self.engines[item[:-3]] = [None,"install/commanded_stt/share/commanded_stt/{}".format(item), True]

		# set listen/stop listen to be active always
		act_st = ActivationState()
		act_st.activate.array.append("heyrobot")
		act_st.activate.array.append("stop_listening")
		act_st.stream_after_activation.append(True)
		act_st.stream_after_activation.append(False)
		self.set_active_listeners(act_st)

	def lock_speech(self, msg):
		self.speech_locked = True

	def unlock_speech(self, msg):
		time.sleep(3)
		self.speech_locked = False
		print("GO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")

	'''
	Set of external methods that can control this node w/o using speech
	'''
	def set_active_listeners(self, msg):
		to_deactivate = msg.deactivate
		to_activate = msg.activate
		stream_after_activation = msg.stream_after_activation

		if len(to_deactivate.array) > 0 and to_deactivate.array[0] == "all":
			arr = self.engines
		else:
			arr = to_deactivate.array
		for activation_name in arr:
			if self.engines[activation_name][0] is not None:
				engine = self.engines[activation_name][0]
				engine.stop()
				self.engines[activation_name][0] = None
				msg = StringArray()
				msg.array.append("heyrobot")
				msg.array.append("")
				self.in_progress_stt_pub.publish(msg)

		activation_time_delay = 0.2
		if len(to_activate.array) > 0 and to_activate.array[0] == "all":
			arr = self.engines
			to_stream = [stream_after_activation.array[0] for i in range(len(arr))]
		else:
			arr = to_activate.array
			to_stream = stream_after_activation
		for i in range(len(arr)):
			activation_name = arr[i]
			stream = to_stream[i]
			filepath = self.engines[activation_name][1]
			self.engines[activation_name][2] = stream

			thread = threading.Thread(target=self.listen_for_wake_words, args=(filepath,activation_name,activation_time_delay))
			thread.daemon = True			# Daemonize thread
			thread.start()

			activation_time_delay += 0.2
			time.sleep(2)

	def stt_triggered_callback(self, msg):
		string = msg.data
		if string == "stop_listening":
			print("auto 6")
			self.end_perpetual_stream()
		elif string == "heyrobot":
			self.start_countdown()
			self.start_perpetual_stream_callback()
	'''
	End of externally-accessible methods
	'''

	def set_perpetual_stt_cb(self, msg):
		self.perpetual_listener = msg.data

	def start_stream_callback(self, activation_notifier):
		thread = threading.Thread(target=self.single_stream_stt.run_terminal, args=(self.receive_single_stream,self.receive_single_in_progress_stream,self.stream_lock, activation_notifier))
		thread.daemon = True			# Daemonize thread
		thread.start()

	def start_perpetual_stream_callback(self):
		thread = threading.Thread(target=self.single_stream_stt.run_terminal_perpetual, args=(self.receive_single_stream,self.receive_single_in_progress_stream,self.stream_lock))
		thread.daemon = True			# Daemonize thread
		thread.start()

	def end_perpetual_stream(self):
		print("auto 5")
		self.single_stream_stt.cancel_perpetual()

	def receive_single_stream(self, text, activation_notifier):
		if self.speech_locked:
			print("speech is locked")
			return 
		print(text)
		self.start_countdown()

		# handle the case in which text was received after the stop listening command was called
		if self.engines["heyrobot"][0] is not None:
			msg = StringArray()
			msg.array.append(activation_notifier)
			msg.array.append(text)
			self.finished_stt_pub.publish(msg)
		else:
			print("could not send text because listener not active")

	def receive_single_in_progress_stream(self, text, activation_notifier):
		if self.speech_locked:
			print("speech is locked")
			return
		print(text)
		if self.single_stream_stt.must_cancel:
			print("auto 2")
			self.auto_cancel_stream()
			return
		self.start_countdown()
		msg = StringArray()
		msg.array.append(activation_notifier)
		msg.array.append(text)
		self.in_progress_stt_pub.publish(msg)

	def listen_for_wake_words(self, word_library, activation_notifier, startdelay):
		def on_prediction(prob):
			pass

		def on_activation():
			if self.speech_locked:
				print("no activation, speech is locked")
				return
			print("ACTIVATION: {}".format(activation_notifier))

			msg = RosString()
			msg.data = activation_notifier
			self.command_triggered_pub.publish(msg)

			if activation_notifier != "heyrobot" and activation_notifier != "stop_listening":
				if self.engines[activation_notifier][2]:
					self.start_stream_callback(activation_notifier)
			elif activation_notifier == "heyrobot":
				if self.engines[activation_notifier][2]:
					self.start_countdown()
					self.start_perpetual_stream_callback()
			else:
				print("auto 4")
				self.end_countdown()
				self.end_perpetual_stream()

		path = sys.argv[1]

		time.sleep(startdelay)
		print("{} ready".format(activation_notifier))

		engine = PreciseEngine('{}/.venv/bin/precise-engine'.format(path), word_library)
		self.engines[activation_notifier][0] = PreciseRunner(engine, on_prediction=on_prediction, on_activation=on_activation,
					  trigger_level=0)
		self.engines[activation_notifier][0].start()

	def start_countdown(self):
		self.countdown_lock.acquire()
		self.countdown = 15
		if self.countdown_method_lock.acquire(blocking=False):
			thread = threading.Thread(target=self.countdown_to_end)
			thread.daemon = True			# Daemonize thread
			thread.start()
		self.countdown_lock.release()

	def end_countdown(self):
		print("countdown ended")
		self.countdown_lock.acquire()
		self.countdown = -1
		self.countdown_lock.release()

	def countdown_to_end(self):
		if self.perpetual_listener:
			self.countdown_method_lock.release()
			return
		print("countdown started")
		while True:
			time.sleep(0.1)
			self.countdown_lock.acquire()
			self.countdown -= 0.1
			print(self.countdown)
			to_break = False
			if self.countdown < 0:
				to_break = True
			self.countdown_lock.release()
			if to_break:
				print("breaking")
				break
		self.countdown_method_lock.release()
		if not self.perpetual_listener:
			print("auto 1")
			self.auto_cancel_stream()
		print("thread countdown ended")

	def auto_cancel_stream(self):
		self.single_stream_stt.auto_cancel_perpetual()
		msg = RosString()
		msg.data = "stop_listening"
		self.command_triggered_pub.publish(msg)

def main():
	rclpy.init()
	controller = Controller()
	rclpy.spin(controller)

if __name__ == '__main__':
	main()
