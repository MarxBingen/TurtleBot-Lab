#!/usr/bin/env python

from TBBase import TBBase
import rospy
import time

if __name__ == '__main__':
	base=TBBase()
	#base.vorwaerts()
	#base.drehe()
	#time.sleep(2)
	#base.drehe()
	while not rospy.is_shutdown():
		p = base.pruefeFelder()
		if p.mitte == 'Frei':
			base.vorwaerts()
		else:
			base.drehe()
		time.sleep(0.5)
