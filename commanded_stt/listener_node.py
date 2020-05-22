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
cwd=os.getcwd()
sys.path.append("{}/commanded_stt/commanded_stt".format(cwd))
from precise.util import activate_notify
from precise_runner import PreciseRunner, PreciseEngine

# google real-time STT
from single_stream_stt import *

class Controller(Node):

	def __init__(self):
		super().__init__('listener_node')

		# setup modified google real-time stt
		self.single_stream_stt = SingleStreamSTT()

		# semaphore for the stream callback
		self.stream_lock = threading.Lock()

		# setup mycroft threads
		thread = threading.Thread(target=self.listen_for_wake_words, args=("wake_word_engines/robotsay.pb","robotsay",0.5))
		thread.daemon = True			# Daemonize thread
		thread.start()

	def start_stream_callback(self, receive_single_stream, receive_single_in_progress_stream, receive_agent_highlight, activation_time):
		thread = threading.Thread(target=self.single_stream_stt.run_terminal, args=(receive_single_stream,receive_single_in_progress_stream,receive_agent_highlight,activation_time,self.stream_lock))
		thread.daemon = True			# Daemonize thread
		thread.start()

	def listen_for_wake_words(self, word_library, activation_notifier, startdelay):
		def on_prediction(prob):
			pass

		def receive_single_stream(text,activation_time):
			print("text {}: {}".format(activation_time, text))
			#self.detected_text_callback(activation_time, text, activation_notifier)

		def receive_single_in_progress_stream(text):
			print("ip_text {}".format(text))
			#self.send_text_to_interface(text,None,activation_notifier, in_progress=True)

		def receive_agent_highlight():
			print("received content")
			#self.toggle_agent_highlight(activation_notifier,content_id)

		def on_activation():
			print("ACTIVATION: {}".format(activation_notifier))
			activation_time = time.time()
			self.start_stream_callback(receive_single_stream, receive_single_in_progress_stream, receive_agent_highlight, activation_time)
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
