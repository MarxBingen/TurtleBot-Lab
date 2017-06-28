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
		bs.set_color(name='yellow')
	while not rospy.is_shutdown():
		p = base.pruefeFelder()
		if p.rechts == 'Frei':
			base.drehe('rechts')
			base.vorwaerts()
		elif p.mitte == 'Frei':
			base.vorwaerts()
		else:
			base.drehe('links')
		detected = base.pruefeObject()
		print detected
		if detected == "Green":
			print "Gruen erkannt"
			for bs in blinkstick.find_all():
				bs.set_color(name='green')
		elif detected == "Red":
			for bs in blinkstick.find_all():
				bs.set_color(name='red')
		else:
			for bs in blinkstick.find_all():
				bs.set_color(name='yellow')
		time.sleep(0.5)

	for bs in blinkstick.find_all():
		bs.set_color(name='off')
