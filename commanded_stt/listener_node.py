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

class Controller(Node):

	def __init__(self):
		super().__init__('listener_node')

		# setup publishers and subscribers
		self.command_triggered_pub = self.create_publisher(String,'/stt/triggered',10)
		self.in_progress_stt_pub = self.create_publisher(StringArray,'/stt/in_progress',10)
		self.finished_stt_pub = self.create_publisher(StringArray,'/stt/result',10)

		# setup modified google real-time stt
		self.single_stream_stt = SingleStreamSTT()

		# semaphore for the stream callback
		self.stream_lock = threading.Lock()

		# setup mycroft threads
		thread = threading.Thread(target=self.listen_for_wake_words, args=("install/commanded_stt/share/commanded_stt/okay-robot.pb","okay-robot",0.5))
		thread.daemon = True			# Daemonize thread
		thread.start()

	def start_stream_callback(self, receive_single_stream, receive_single_in_progress_stream):
		thread = threading.Thread(target=self.single_stream_stt.run_terminal, args=(receive_single_stream,receive_single_in_progress_stream,self.stream_lock))
		thread.daemon = True			# Daemonize thread
		thread.start()

	def listen_for_wake_words(self, word_library, activation_notifier, startdelay):
		def on_prediction(prob):
			pass

		def receive_single_stream(text):
			print("text {}".format(text))
			msg = StringArray()
			msg.array.append(activation_notifier)
			msg.array.append(text)

			self.finished_stt_pub.publish(msg)
			#self.detected_text_callback(activation_time, text, activation_notifier)

		def receive_single_in_progress_stream(text):
			print("ip_text {}".format(text))
			#self.send_text_to_interface(text,None,activation_notifier, in_progress=True)

		def on_activation():
			print("ACTIVATION: {}".format(activation_notifier))
			self.start_stream_callback(receive_single_stream, receive_single_in_progress_stream)
			#self.detected_text_callback(activation_time, text, activation_notifier)
			#activate_notify()

		path = sys.argv[1]

		time.sleep(startdelay)

		engine = PreciseEngine('{}/.venv/bin/precise-engine'.format(path), word_library)
		PreciseRunner(engine, on_prediction=on_prediction, on_activation=on_activation,
					  trigger_level=0).start()

def main():
	rclpy.init()
	controller = Controller()
	rclpy.spin(controller)

if __name__ == '__main__':
	main()
