#!/usr/bin/env python

from TBBase import TBBase
import rospy
from TBRandom import zufall
import time
import cv2

from blinkstick import blinkstick

if __name__ == '__main__':
	base=TBBase(0.40)
	for bs in blinkstick.find_all():
		bs.set_color(name='green')
	#base.vorwaerts()
	#base.drehe() #
	#time.sleep(2)
	#base.drehe()
	while not rospy.is_shutdown():
		p = base.pruefeFelder()
		z = zufallBelegung(p)
		if (z=='mitte'):
			base.vorwaerts()
		elif (z=='sackgasse'):
			base.drehe('links')
		else:
			base.drehe(z)
			base.vorwaerts()
		time.sleep(0.5)
	for bs in blinkstick.find_all():
		bs.set_color(name='red')
