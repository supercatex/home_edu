#! /usr/bin/env python


class Astra(object):

	def __init__(self, name):
		self.name = name
	
	def rgb_image_raw(self):
		return "/%s/rgb/image_raw" % self.name
	
	def depth_image_raw(self):
		return "/%s/depth/image_raw" % self.name
