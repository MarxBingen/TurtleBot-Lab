#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from collections import namedtuple

class WallDetection:

	laserSub = None
	lastScan = None
	#definition WallInfo
	WallInfo = namedtuple('WallInfo','links,mitte,rechts')
	lastWallInfo = WallInfo('Frei','Frei','Frei')
	#wand annaeherung links oder rechts
	wallToClose = ''

	def __init__(self):
		print "Wanderkennung gestartet"
		self.laserSub = rospy.Subscriber('/scan', LaserScan,queue_size = 1,callback=self.laserCallback)

	def wallGetsCloser(self):
		return self.wallToClose

	def laserCallback(self,data):
		if (rospy.is_shutdown() and not self.laserSub is None):
			self.laserSub.unregister()
			return
		self.lastScan = data

	def detectWalls(self):
		#vorne  380 - 420 = 40
		#links  690 - 805 = 130
		#links annaehreung 615-730
		#rechts 15 - 140 = 130
		#rechts annaeherung 50-165
		leftC = 0
		centerC = 0
		rightC = 0
		#Counter fuer zu nah dran
		leftClose = 0
		rightClose = 0
		maxIndex = len(self.lastScan.ranges)
		for x in range(0, maxIndex):
			if (x > 690 and x < 805):
				if (self.lastScan.ranges[x]<0.40):
					leftC = leftC + 1
			if (x > 615 and x < 750):
				if (self.lastScan.ranges[x]<0.20):
					leftClose = leftClose + 1
			if (x > 380 and x < 420):
				if (self.lastScan.ranges[x]<0.15):
					centerC = centerC + 1
			if (x > 15 and x < 140):
				if (self.lastScan.ranges[x]<0.40):
					rightC = rightC + 1
			if (x > 50 and x < 165):
				if (self.lastScan.ranges[x]<0.20):
					rightClose = rightClose + 1
		left = 'Frei' if leftC<60 else 'Belegt'
		right = 'Frei' if rightC<60 else 'Belegt'
		center = 'Frei' if centerC < 5 else 'Belegt'
		#muss vertauscht werden, da scanner ueber kopf
		self.lastWallInfo = self.WallInfo(right,center,left)
		#wand annaeherung 20 lasermessungen zu nah...dann annaehrung
		if (rightClose > 10 or leftClose > 10):
			if (rightClose > leftClose):
				self.wallToClose = 'links'
			else:
				self.wallToClose = 'rechts'
		else:
			self.wallToClose = ''
		#print leftC,centerC,rightC
		#print leftClose, rightClose
		return self.lastWallInfo

