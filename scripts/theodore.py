#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from core import Dynamixel
from core import Kobuki
from core import Astra
import math
import speech_recognition as sr
import numpy as np
import pyttsx3
import re
import datetime
import time
#define
ARM_LENGTH = 10.5
chassis = Kobuki()
camera = Astra()
MAX_SPEED = 1.0
L1 ,L2= ARM_LENGTH, ARM_LENGTH
r = sr.Recognizer()
now = datetime.datetime.now()
findquestion = ""

question =	{
  "handsome": "I think that Justin Trudeau is very handsome",
  "time zones": "Canada spans almost ten million square km and comprises six time zones",
  "longest street": "Yonge Street in Ontario is the longest street in the world",
  "exported from":"The bear cub was named Winnipeg. It inspired thestories of Winnie the Pooh",
  "blackberry":"It was developed in Ontario, at Research In Motion’s Waterloo offices.",
  "largest coin":"The Big Nickel in Sudbury, Ontario. It is nine meters in diameter.",
  "invaded by":"The first time that the USA invaded Canada was in seventeen seventy five.The USA invaded Canada a second time in sighteen and twelve.",
  "gold medals":"Canada does! With fourteen Golds at the twenty ten VancouverWinter Olympics.",
  "Yonge Street":"Yonge street is almost two thousand km, starting at Lake Ontario, and running north to the Minnesota border",
  "coined the term":"Sandy Gardiner, a journalist of the Ottawa Journal.",
  "Canada named Canada":"French explorers misunderstood the local nativeword Kanata, which means village.",
  "Mounted Police":"The Mounted Police was formed in eighteen seventy three.",
  "Royal Canadian":"In nighteen twenty, when The Mounted Police merged with the Dominion Police.",
  "how big is":"Today, the RCMP has close to thirty thousand members.",
  "montreal":"Montreal is often called the City of Saints or the City of a Hundred Bell Towers.",
  "located":"The Hotel de Glace is in Quebec.",
  "ice are required":"The Hotel de Glace requires about four hundred tons of ice.",
  "snow":"Every year, twelve thousand tons of snow are used for The Hotel de Glace.",
  "in summer":"No. Every summer it melts away, only to be rebuilt the following winter.",
  "Where is Canada":"Canada’s only desert is British Columbia.",
  "only desert":"The British Columbia desert is only fifthteen miles long.",
  "famous male":"Leonard Cohen, Keanu Reeves, and Jim Carrey.",
  "famous female":"Celine Dion, Pamela Anderson, and Avril Lavigne.",
  "origin of":"Comic Sans is based on Dave Gibbons’ lettering in the Watchmen comic books.",
  "nanobot":"The smallest robot possible is called a nanobot.",
  "small can":"A nanobot can be less than one-thousandth of amillimeter.",
  "an award by":"The Academy thought that Tron cheated by using computers.",
  "disk drive":"The IBM three o five RAMAC.",
  "drive launched":"The IBM three o five RAMAC was launched in nighteen fifthty six.",
  "first hard disk":"The IBM three o five RAMAC hard disk weighed over a ton and stored five megabytes of data.",
  "stands for":"CAPTCHA is an acronym for Completely Automated Public Turing test to tell Computers and Humans Apart",
  "computer bug":"The first actual computer bug was a dead moth stuck in a Harvard Mark two.",
  "robots on Mars":"There are four robots on Mars include Sojourner, Spirit,Opportunity, and Curiosity. Three more crashed on landing.",
  "first android":"Professor Kevin Warwick uses chips in his arm to operate doors, a robotic hand, and a wheelchair.",
  "mechanical":"A robot sketch made by Leonardo DaVinci.",
  "in pass the":"Some people think it was IBM Watson, but it was Eugene, a computer designed at England’s University of Reading.",
  "paradox state":"Moravec’s paradox states that a computer can crunch numbers like Bernoulli, but lacks a toddler’s motor skills.",
  "knowledge engineering":"It is when you need to load an AI with enough knowledge to start learning.",
  "is worried about":"I don’t know. He should worry more about the people’s impact on humanity.",
  "a threat":"No. Humans are the real threat to humanity.",
  "chatbot":"A chatbot is an A.I. you put in customer service to avoid paying salaries.",
  "cars safe":"Yes. Car accidents are product of human misconduct.",
  "the compiler":"Grace Hoper. She wrote it in her spare time.",
  "Programming Language":"C was invented by Dennis MacAlistair Ritchie.",
  "created the Python":"Python was invented by Guido van Rossum.",
  "zuckerberg a robot":"Sure. I’ve never seen him drink water.",
  "the Apple":"My lord and master Steve Wozniak.",
  "computer programmer":"Ada Lovelace.",
  "use to open":"Adobe Wan Kenobi.",
}

def speaktotext():
	mic = sr.Microphone(device_index=6)
	print(mic)
	
	with sr.Microphone() as source: 
		print("Loading Microphone...") 
		#load microphone
		print("Say something!")
		speak("you can ask me now")
		audio = r.listen(source)

	try:
		text = r.recognize_google(audio, language="en-US")
		# time.sleep(8)
		print(text)
		print('you didnt say hello')
		#print(text)
		return text

	except sr.UnknownValueError:
		print("Google Speech Recognition could not understand audio")
		text = "Google Speech Recognition could not understand audio"
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
	try:
		for key in question.keys():
			if key in text:
				speak(question[key])

	except:
		print("ask again")
		speak("sorry")
		speak("pardon")

def speak(word):
	engine.setProperty('rate', 120)
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

def follower_line(dis):

	while(1):
		#rgb = camera.rgb_image
		depth = camera.depth_image
		#activate the task
		if (0<depth[240 , 320]<60):
			startfollow = 1
		#start following/"

		while(depth[240,320]>dis and startfollow == 1):
			chassis.move(0.5,0)

if __name__ == "__main__":
	rospy.init_node("home_edu_arm", anonymous=True)
	engine = pyttsx3.init()
	engine.setProperty('rate', 100)
	engine.say('hello, my name is mustar')
	engine.runAndWait()
	listening= 1
	while (listening == 1):
		text = speaktotext()
		print(text)
		autoanswor(text)
	
	shoulder = Dynamixel("shoulder_controller")
	
	elbow = Dynamixel("elbow_controller")

	shoulder.set_speed(MAX_SPEED)
	
	elbow.set_speed(MAX_SPEED)
	go_to_coordinate(0,14)