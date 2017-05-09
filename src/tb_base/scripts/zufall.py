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
	#base.vorwaerts()
	#base.drehe() #
	#time.sleep(2)
	#base.drehe()
	while not rospy.is_shutdown():
		p = base.pruefeFelder()
		if p.rechts == 'Frei':
			base.drehe('rechts')
			base.vorwaerts()
		elif p.mitte == 'Frei':
			base.vorwaerts()
		elif p.links == 'Frei':
			base.drehe('links')
			base.vorwaerts()
		else:
			base.drehe('rechts')
		time.sleep(0.5)
	for bs in blinkstick.find_all():
		bs.set_color(name='red')
