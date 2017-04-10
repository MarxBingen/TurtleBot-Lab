#!/usr/bin/env python

from TBBase import TBBase
import rospy
from random import randint
import time
import cv2

if __name__ == '__main__':
	base=TBBase(0.35)
	#base.vorwaerts()
	#base.drehe()
	#time.sleep(2)
	#base.drehe()
	while not rospy.is_shutdown():
		p = base.pruefeFelder()
		if p.rechts == 'Frei' and p.links == 'Frei':
			if p.mitte == 'Frei':
				maxL=3
			else:
				maxL=2
			rl = randint(0,maxL)
			print "Random:", rl
			if rl == 0:
				base.drehe('rechts')
			elif rl == 1:
				base.drehe('links')
			else:
				base.vorwaerts()
		elif p.rechts == 'Frei':
			base.drehe('rechts')
			base.vorwaerts()
		elif p.mitte == 'Frei':
			base.vorwaerts()
		else:
			base.drehe('links')
		time.sleep(0.5)
