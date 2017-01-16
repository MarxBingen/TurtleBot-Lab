#!/usr/bin/env python

from TBBase import TBBase
import rospy
import time

if __name__ == '__main__':
	base=TBBase()
	#base.vorwaerts()
	base.drehe()
	time.sleep(2)
	base.drehe()
