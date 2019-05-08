#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from Dynamixel import Dynamixel
import math
import speech_recognition as sr
import numpy as np
ARM_LENGTH = 10.5
MAX_SPEED = 1.0
import pyttsx3
import re
import datetime
import time


L1 = ARM_LENGTH
L2 = ARM_LENGTH
r = sr.Recognizer()
now = datetime.datetime.now()
def speaktotext():
	mic = sr.Microphone(device_index=6)
	print(mic)
	
	with sr.Microphone() as source: 
		print("Loading Microphone...") 
		#load microphone
		print("Say something!")
		audio = r.listen(source)

	try:
		text = r.recognize_google(audio, language="en-US")
		time.sleep(2)
		print(text)
		if (bool(re.search("hello", text)) == True):
			print('you said hello')
			engine.setProperty('rate', 90)
			engine.say('you said hello ');
			engine.runAndWait();
		print('you didnt say hello')
		#print(text)
		return text

	except sr.UnknownValueError:
		print("Google Speech Recognition could not understand audio")
		return text
	except sr.RequestError as e:
		print("No response from Google Speech Recognition service: {0}".format(e))
		return text
	except Exception as e:
		print('excepted!', e)
		text = "whatyouspeak?"
		return text


def autoanswor(text):
	if "Google Speech Recognition could not understand audio" in text:
		speak("i do not understand")


	if "how are you" in text:
		speak("I am fine")

	if "what time is it" in text:
		
		speak(str(now.hour))
		speak(str(now.minute))

	if "hello" in text:
		speak("hello")

	if "name" in text:
		speak("my name is robot")

def speak(word):
	engine.setProperty('rate', 80)
	engine.say(word);
	engine.runAndWait();
        

def deg2rad(deg):
	rad = math.radians(deg)
	return rad
	
def gohome():
	shoulder.set_radian(0)
	elbow.set_radian(0)
	
def go_to_coordinate(x,y):
	if (x*x + y*y < 10.5*10.5):
		print("OK LA")
	
	d = math.sqrt(x*x + y*y)
	theta1 = math.acos((L1*L1+d*d-L2*L2)/(2*L1*d))
	rotate_a = math.pi / 2 - math.atan2(y,x) - theta1
	theta2 = math.acos((L1*L1 + L2*L2 - d*d)/(2*L1*L2))
	rotate_b = math.pi - theta2
	shoulder.set_radian(rotate_a)
	elbow.set_radian(rotate_b)

	
if __name__ == "__main__":
	rospy.init_node("home_edu_arm", anonymous=True)


	engine = pyttsx3.init();
	engine.setProperty('rate', 100)
	engine.say('please say hello');
	engine.runAndWait();
	listening= 1
	while (listening == 1):
		text = speaktotext()
		autoanswor(text)

	'''engine = pyttsx3.init();
	engine.setProperty('rate', 100)
	engine.say(text);
	engine.runAndWait();'''
	

	MAX_SPEED = 1.2
	
	shoulder = Dynamixel("shoulder_controller")
	
	elbow = Dynamixel("elbow_controller")

	shoulder.set_speed(MAX_SPEED)
	
	elbow.set_speed(MAX_SPEED)
	go_to_coordinate(0,14)

	