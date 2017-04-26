#!/usr/bin/env python

from TBBase import TBBase
import rospy
from TBRandom import zufall
import time
import cv2

if __name__ == '__main__':
	base=TBBase(0.35)
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
		time.sleep(0.5)
