#!/usr/bin/env python
import pyttsx3


class Speaker(object):

	def __init__(self, speed=100, volume=1.0, voice_id=2):
		self.engine = pyttsx3.init()
		self.set_speed(speed)
		self.set_volume(volume)
		self.set_voice(voice_id)
	
	def set_speed(self, speed):
		self.engine.setProperty("rate", speed)
	
	def set_volume(self, volume):
		self.engine.setProperty("volume", volume)
	
	def set_voice(self, voice_id):
		voices = self.engine.getProperty("voices")
		#print(len(voices))
		if voice_id >= 0 and voice_id < len(voices):
			voice = voices[voice_id]
			self.engine.setProperty("voice", voice.id)
		else:
			print("No voice id: ", voice_id)
	
	def say(self, msg, log=False):
		print(msg)
		self.engine.say(msg)
		self.engine.runAndWait()


if __name__ == "__main__":
	P = Speaker(180)
	P.say("Good afternoon, what's your name, I am raspberry pi, I am your best mini computer that can do a lot of things")
	# 2, 9, 10, 11, 14, 15, 17, 22
