#!/usr/bin/env python
import speech_recognition 
import rospy
from std_msgs.msg import String
import time

'''
https://pypi.org/project/PyAudio/#files
cd PyAudio-0.2.11
sudo python setup.py install

/usr/share/alsa/alsa.conf
#pcm.rear cards.pcm.rear
#pcm.center_lfe cards.pcm.center_lfe
#pcm.side cards.pcm.side
#pcm.surround21 cards.pcm.surround21
#pcm.surround40 cards.pcm.surround40
#pcm.surround41 cards.pcm.surround41
#pcm.surround50 cards.pcm.surround50
#pcm.surround51 cards.pcm.surround51
#pcm.surround71 cards.pcm.surround71
'''


class Speech2Text(object):

	def __init__(self, lang="en-US", topic="/home_edu/facial"):
		self.lang = lang
		self.recognizer = speech_recognition.Recognizer()  
		self.publisher =     rospy.Publisher(
			topic,
			String,
			queue_size=1,
			latch=True
		)

	def ambient_noise(self):
		with speech_recognition.Microphone() as source:
			self.recognizer.adjust_for_ambient_noise(source)

	def listen(self, msg1="listening", msg2="processing", msg3="finished", f1="smiling", f2="suspicious", f3="smart", keep_message=False):
		try:
			with speech_recognition.Microphone() as source:
				cmd = f1 + ":" + msg1
				self.publisher.publish(cmd)
				audio = self.recognizer.listen(source)
				print("Got the audio.")
			cmd = f2 + ":" + msg2
			self.publisher.publish(cmd)
			text = self.recognizer.recognize_google(audio, language=self.lang)
			cmd = f3 + ":" + text
			self.publisher.publish(cmd)
			return text
		except Exception as e:
			print(e)
			return ""



if __name__ == "__main__":
	s = Speech2Text()
	s.ambient_noise()
	while True:
		print("ready")
		t = s.listen()
		print(t)
		if t == "goodbye":
			break
	print("bye-bye")
