#!/usr/bin/env python

from TBBase import TBBase
import rospy
from TBRandom import zufall
import time
import cv2

from blinkstick import blinkstick

if __name__ == '__main__':
	base=TBBase(0.38)
	for bs in blinkstick.find_all():
		bs.set_color(name='green')
	while not rospy.is_shutdown():
		base.drehe('links')
		detected = base.pruefeObject()
		if detected == True:
			print "Bild erkannt"
			for bs in blinkstick.find_all():
				bs.set_color(name='green')
		else:
			for bs in blinkstick.find_all():
				bs.set_color(name='red')
		time.sleep(0.5)

	for bs in blinkstick.find_all():
		bs.set_color(name='off')
