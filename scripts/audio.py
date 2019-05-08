#!/usr/bin/env python

# -*- coding: utf-8 -*-

import speech_recognition as sr

lists = sr.Microphone.list_microphone_names()

print(lists)
print('----------------------------')


mic = sr.Microphone(device_index=6)
print(mic)

#obtain audio from the microphone
r=sr.Recognizer() 
with sr.Microphone() as source:
    print("Please wait. Calibrating microphone...") 
    #listen for 5 seconds and create the ambient noise energy level 
    r.adjust_for_ambient_noise(source, duration=5) 
    print("Say something!")
    audio=r.listen(source)

# recognize speech using Google Speech Recognition 
try:
    print("Google Speech Recognition thinks you said:")
    print(r.recognize_google(audio, language="en-US"))
except sr.UnknownValueError:
    print("Google Speech Recognition could not understand audio")
except sr.RequestError as e:
    print("No response from Google Speech Recognition service: {0}".format(e))
