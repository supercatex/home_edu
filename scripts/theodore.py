#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from Dynamixel import Dynamixel
import math
import speech_recognition
import numpy as np
ARM_LENGTH = 10.5
MAX_SPEED = 1.0
import pyttsx3


L1 = ARM_LENGTH
L2 = ARM_LENGTH

def speaktotext():
	
	r = speech_recognition.Recognizer()
	with speech_recognition.Microphone() as source: 
	
		print("start speak")
		r.adjust_for_ambient_noise(source)#shut the voice
		audio = r.listen(source)

	try:
		Text = r.recognize_google(audio, language="en-US")
		print(TEXT)
		return TEXT
	except:
		Text = "whatyouspeak?"



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

	TEXT = speaktotext()
	print (TEXT)
	
	engine = pyttsx3.init();
	engine.setProperty('rate', 100)
	engine.say(TEXT);
	engine.runAndWait();
	

	MAX_SPEED = 1.2
	
	shoulder = Dynamixel("shoulder_controller")
	
	elbow = Dynamixel("elbow_controller")

	shoulder.set_speed(MAX_SPEED)
	
	elbow.set_speed(MAX_SPEED)
	go_to_coordinate(0,14)

	