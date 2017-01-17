#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from collections import namedtuple

class WallDetection:

	lastScan = LaserScan()
	laserSub = None

	def __init__(self):
		print "Wanderkennung gestartet"
		self.laserSub = rospy.Subscriber('/scan', LaserScan,self.laserCallback)


	def detectWalls(self,rasterSize=0.5):
		#vorne  380 - 420 = 40
		#links  680 - 810 = 130
		#rechts 1 - 130 = 130
		leftC = 0
		centerC = 0
		rightC = 0
		for l in range (680,810):
			if (self.lastScan.ranges[l]<rasterSize):
				leftC = leftC + 1
		for c in range(380,420):
			if (self.lastScan.ranges[c]<(rasterSize/2)):
				centerC = centerC + 1
		for r in range(1,130):
			if (self.lastScan.ranges[r]<rasterSize):
				rightC = rightC + 1
		left = 'Frei' if leftC<20 else 'Belegt'
		right = 'Frei' if rightC<20 else 'Belegt'
		center = 'Frei' if centerC < 5 else 'Belegt'
		WallInfo = namedtuple('WallInfo','links,mitte,rechts')
		#muss vertauscht werden, da scanner ueber kopf
		result = WallInfo(right,center,left)
		return result

	def laserCallback(self,data):
		if (rospy.is_shutdown() and not self.laserSub is None):
			self.laserSub.unregister()
			return
		self.lastScan=data

