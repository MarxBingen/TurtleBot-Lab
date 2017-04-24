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
		v = []
		if p.rechts == 'Frei':
			v.append('rechts')
		if p.links == 'Frei':
			v.append('links')
		if p.mitte == 'Frei':
			v.append('mitte')
		z = zufall(v)
		if (z=='mitte'):
			base.vorwaerts()
		else:
			base.drehe(z)
			base.vorwaerts()
		time.sleep(0.5)
	for bs in blinkstick.find_all():
		bs.set_color(name='red')
