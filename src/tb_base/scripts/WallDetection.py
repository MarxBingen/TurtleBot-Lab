#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PolygonStamped,Point32
from collections import namedtuple

class WallDetection:

	laserSub = None
	lastScan = None
	#definition WallInfo
	WallInfo = namedtuple('WallInfo','links,mitte,rechts')
	lastWallInfo = WallInfo('Frei','Frei','Frei')
	#wand annaeherung links oder rechts
	wallToClose = ''
	#linkes feld
	FeldInfo = namedtuple('FeldInfo','x1,x2,y1,y2')
	frontArea = FeldInfo(0.0,0.35,-0.15,0.15)
	leftArea = FeldInfo(-0.35,0.0,-0.55,-0.15)
	rightArea = FeldInfo(-0.35,0.0,0.15,0.55)

	def __init__(self):
		print "Wanderkennung gestartet"
		self.laserSub = rospy.Subscriber('/scan', LaserScan,queue_size = 1,callback=self.laserCallback)
		self.areaLeft = rospy.Publisher('areaLeft', PolygonStamped, queue_size=1)
		self.areaFront = rospy.Publisher('areaFront', PolygonStamped, queue_size=1)
		self.areaRight = rospy.Publisher('areaRight', PolygonStamped, queue_size=1)

	def wallGetsCloser(self):
		return self.wallToClose

	def publishAreas(self):
		#front
		front = [Point32(y=-0.15),Point32(y=0.15),Point32(x=0.35,y=0.15),Point32(x=0.35,y=-0.15)]
		left = [Point32(y=-0.15),Point32(y=-0.55),Point32(x=-0.35,y=-0.55),Point32(x=-0.35,y=-0.15)]
		right = [Point32(y=0.15),Point32(y=0.55),Point32(x=-0.35,y=0.55),Point32(x=-0.35,y=0.15)]
		p = PolygonStamped()
		p.header.frame_id="laser"
		p.header.stamp = rospy.Time.now()
		p.polygon.points=front
		self.areaFront.publish(p)
		p = PolygonStamped()
		p.header.frame_id="laser"
		p.header.stamp = rospy.Time.now()
		p.polygon.points=left
		self.areaLeft.publish(p)
		p = PolygonStamped()
		p.header.frame_id="laser"
		p.header.stamp = rospy.Time.now()
		p.polygon.points=right
		self.areaRight.publish(p)

	def laserCallback(self,data):
		if (rospy.is_shutdown() and not self.laserSub is None):
			self.laserSub.unregister()
			return
		self.publishAreas()
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

