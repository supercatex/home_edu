#!/usr/bin/env python

import cv2 as cv

from core import Astra as astra
import rospy

import data


rospy.init_node("pcms_test", anonymous=True)

a = 'What program do Jedi use to open PDF file'

b = "Jedi PDF files"

b = b.split(" ")

count = 1

for i in b:
    if i in a:
        count += 1
        print(count)

if len(b) == count:
    print("String a matches string b")

else:
    print("String a doesn't match string b")

rate = rospy.Rate(20)
