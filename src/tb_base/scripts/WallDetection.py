#!/usr/bin/env python

import rospy
import math
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
	frontArea = FeldInfo(0.0  , 0.15,-0.10, 0.10)
	leftArea =  FeldInfo(-0.25,-0.0,-0.30,-0.18)
	rightArea = FeldInfo(-0.25,-0.0, 0.18, 0.30)
	frontLeftArea = FeldInfo(0.0,0.2,-0.18,-0.10)
	frontRightArea = FeldInfo(0.0,0.2,0.10,0.18)

	anglesCalculated = False
	sinAngles = []
	cosAngles = []

	def __init__(self):
		print "Wanderkennung gestartet"
		self.areaLeft = rospy.Publisher('areaLeft', PolygonStamped, queue_size=1)
		self.areaFront = rospy.Publisher('areaFront', PolygonStamped, queue_size=1)
		self.areaRight = rospy.Publisher('areaRight', PolygonStamped, queue_size=1)
		self.areaFRight = rospy.Publisher('areaFRight', PolygonStamped, queue_size=1)
		self.areaFLeft = rospy.Publisher('areaFLeft', PolygonStamped, queue_size=1)
		self.laserSub = rospy.Subscriber('/scan', LaserScan,queue_size = 1,callback=self.laserCallback)

	def calcAngles(self, numAngles,minAngle,angleIncrement):
		self.anglesCalculated = True
		for i in range(numAngles):
			self.sinAngles.append(math.sin(minAngle+(angleIncrement*i)))
			self.cosAngles.append(math.cos(minAngle+(angleIncrement*i)))
		print "Sin/Cos-Tabellen erstellt"

	def wallGetsCloser(self):
		return self.wallToClose

	def publishAreas(self):
		#front
		fa = self.frontArea
		front = [Point32(y=fa.y1),Point32(y=fa.y2),Point32(x=fa.x2,y=fa.y2),Point32(x=fa.x2,y=fa.y1)]
		la = self.leftArea
		left = [Point32(x=la.x1,y=la.y1),Point32(x=la.x2,y=la.y1),Point32(x=la.x2,y=la.y2),Point32(x=la.x1,y=la.y2)]
		ra = self.rightArea
		right = [Point32(x=ra.x1,y=ra.y1),Point32(x=ra.x2,y=ra.y1),Point32(x=ra.x2,y=ra.y2),Point32(x=ra.x1,y=ra.y2)]
		fla = self.frontLeftArea
		fra = self.frontRightArea
		fleft = [Point32(x=fla.x1,y=fla.y1),Point32(x=fla.x1,y=fla.y2),Point32(x=fla.x2,y=fla.y2),Point32(x=fla.x2,y=fla.y1)]
		fright = [Point32(x=fra.x1,y=fra.y1),Point32(x=fra.x1,y=fra.y2),Point32(x=fra.x2,y=fra.y2),Point32(x=fra.x2,y=fra.y1)]
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
		p = PolygonStamped()
		p.header.frame_id="laser"
		p.header.stamp = rospy.Time.now()
		p.polygon.points=fright
		self.areaFRight.publish(p)
		p = PolygonStamped()
		p.header.frame_id="laser"
		p.header.stamp = rospy.Time.now()
		p.polygon.points=fleft
		self.areaFLeft.publish(p)

	def laserCallback(self,data):
		if (rospy.is_shutdown() and not self.laserSub is None):
			self.laserSub.unregister()
			return
		self.publishAreas()
		self.lastScan = data
		if self.anglesCalculated == False:
			self.calcAngles(len(data.ranges),data.angle_min,data.angle_increment)
		self.internal_detectWalls2(data)

	#def detectWalls2(self):
	#	return self.lastWallInfo

	def internal_detectWalls2(self,data):
		if self.anglesCalculated == False:
			print "No Scan recieved"
			return
		counter = 0
		#performance verbessern durch funktions-referenzen
		p2k = self.polar2Koord
		pIr = self.pointInRect
		#counter fuer Felder
		inLeft  = 0
		inFront = 0
		inRight = 0
		inFrontLeft = 0
		inFrontRight = 0
		for r in data.ranges:
			x,y = p2k(counter,r)
			if (pIr(x,y,self.leftArea)):
				inLeft = inLeft + 1
			if (pIr(x,y,self.frontArea)):
				inFront = inFront + 1
			if (pIr(x,y,self.rightArea)):
				inRight = inRight + 1
			if (pIr(x,y,self.frontRightArea)):
				inFrontRight = inFrontRight + 1
			if (pIr(x,y,self.frontLeftArea)):
				inFrontLeft = inFrontLeft + 1
			counter = counter + 1
		left  = 'Frei' if inLeft  < 10 else 'Belegt'
		right = 'Frei' if inRight < 10 else 'Belegt'
		front = 'Frei' if inFront < 10 else 'Belegt'
		#print inFrontLeft,inFrontRight
		if ((inFrontLeft > 10) != (inFrontRight > 10)):
			if (inFrontLeft > inFrontRight):
				self.wallToClose = 'links'
			else:
				self.wallToClose = 'rechts'
		else:
			self.wallToClose = ''
		self.lastWallInfo = self.WallInfo(left,front,right)
		#print self.lastWallInfo, self.wallToClose

	def polar2Koord(self,angleIndex,range):
		x = range * self.cosAngles[angleIndex]
		y = range * self.sinAngles[angleIndex]
		return x,y

	def pointInRect(self,x,y,rect):
		if x > rect.x1 and x < rect.x2:
			if y > rect.y1 and y < rect.y2:
				return True
		return False


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

