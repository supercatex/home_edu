#!/usr/bin/env python
import rospy
import speech_recognition


'''
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
        print("ready")
        t = s.listen()
        print(t)
	if t == "goodbye":
		break
    print("bye-bye")


