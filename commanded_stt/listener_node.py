import rclpy
from rclpy.node import Node

# mycroft
import sys
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
from std_msgs.msg import String
from wisc_ros2_msgs.msg import StringArray
from wisc_ros2_msgs.msg import ActivationState

class Controller(Node):

	def __init__(self):
		super().__init__('listener_node')

		# setup publishers and subscribers
		self.command_triggered_pub = self.create_publisher(String,'/stt/triggered',10)
		self.in_progress_stt_pub = self.create_publisher(StringArray,'/stt/in_progress',10)
		self.finished_stt_pub = self.create_publisher(StringArray,'/stt/result',10)

		self.external_trigger_sub = self.create_subscription(String,'/stt/trigger',self.stt_triggered_callback,10)
		self.set_listeners_sub = self.create_subscription(ActivationState, "/stt/set_active", self.set_active_listeners, 10)

		# communication with the robot
		self.update_wc_pub = self.create_publisher(WebContent, "/robot/web_content", 10)

		# setup modified google real-time stt
		self.single_stream_stt = SingleStreamSTT()

		# semaphore for the stream callback
		self.stream_lock = threading.Lock()

		# store active engines
		self.engines = {}
		for item in os.listdir("src/install/commanded_stt/share/commanded_stt"):
			if len(item) > 3 and item[-3:] == ".pb":
				self.engines[item[:-3]] = [None,"src/install/commanded_stt/share/commanded_stt/{}".format(item)]

	def set_active_listeners(self, msg):
		to_deactivate = msg.deactivate
		to_activate = msg.activate

		for activation_name in to_deactivate.array:
			engine = self.engines[activation_name][0]
			engine.stop()

		activation_time_delay = 0.2
		for activation_name in to_activate.array:
			filepath = self.engines[activation_name][1]

			thread = threading.Thread(target=self.listen_for_wake_words, args=(filepath,activation_name,activation_time_delay))
			thread.daemon = True			# Daemonize thread
			thread.start()

			activation_time_delay += 0.2

	def stt_triggered_callback(self, msg):
		string = msg.data
		self.start_stream_callback("auto")

	def start_stream_callback(self, activation_notifier):
		thread = threading.Thread(target=self.single_stream_stt.run_terminal, args=(self.receive_single_stream,self.receive_single_in_progress_stream,self.stream_lock, activation_notifier))
		thread.daemon = True			# Daemonize thread
		thread.start()

	def receive_single_stream(self, text, activation_notifier):
		print(text)
		msg = StringArray()
		msg.array.append(activation_notifier)
		msg.array.append(text)
		self.finished_stt_pub.publish(msg)

	def receive_single_in_progress_stream(self, text, activation_notifier):
		print(text)
		msg = StringArray()
		msg.array.append(activation_notifier)
		msg.array.append(text)
		self.in_progress_stt_pub.publish(msg)

	def update_wc_helper(self, text, color="rgb(75, 15, 0)"):
		wc = WebContent()
		wc.prompt = "<SAME>"
		wc.utt = text
		wc.content = "<SAME>"
		wc.options = "<SAME>"
		wc.color = color

		self.update_wc_pub.publish(wc)

	def listen_for_wake_words(self, word_library, activation_notifier, startdelay):
		def on_prediction(prob):
			pass

		def on_activation():
			print("ACTIVATION: {}".format(activation_notifier))

			# other activations DO require listening for further input
			else:
				msg = String()
				msg.data = activation_notifier
				self.command_triggered_pub.publish(msg)
				self.start_stream_callback(activation_notifier)

		path = sys.argv[1]

		time.sleep(startdelay)
		print("{} ready".format(activation_notifier))

		engine = PreciseEngine('{}/.venv/bin/precise-engine'.format(path), word_library)
		self.engines[activation_notifier] = engine
		PreciseRunner(engine, on_prediction=on_prediction, on_activation=on_activation,
					  trigger_level=0).start()

def main():
	rclpy.init()
	controller = Controller()
	rclpy.spin(controller)

if __name__ == '__main__':
	main()
