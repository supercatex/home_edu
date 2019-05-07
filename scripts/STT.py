#!/usr/bin/env python
import rospy
import speech_recognition


class Speech2Text(object):

    def __init__(self, lang="en-US"):
        self.lang = lang
	self.recognizer = speech_recognition.Recognizer()

    def ambient_noise(self):
        with speech_recognition.Microphone() as source:
            self.recognizer.adjust_for_ambient_noise(source)

    def listen(self):
        try:
            with speech_recognition.Microphone() as source:
                audio = self.recognizer.listen(source)
            text = self.recognizer.recognize_google(audio, language=self.lang)
            return text
        except Exception as e:
            print(e)
            return ""


if __name__ == "__main__":
    s = Speech2Text()
    s.ambient_noise()
    while True:
        t = s.listen()
        print(t)
	if t == "goodbye":
            break
    print("bye-bye")


