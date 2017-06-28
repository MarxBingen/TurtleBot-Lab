#!/usr/bin/env python

import rospy
import math
import time
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PolygonStamped,Point32
from collections import namedtuple
from laser_geometry import LaserProjection

def pointInRect(x,y,rect):
	if x > rect.x1 and x < rect.x2:
		if y > rect.y1 and y < rect.y2:
			return True
	return False



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

	frontHelpArea = FeldInfo(0.15,0.2,-0.1,0.1)
	frontLeftArea = FeldInfo(0.0,0.2,-0.18,-0.10)
	frontRightArea = FeldInfo(0.0,0.2,0.10,0.18)

	anglesCalculated = False
	cos_sin_map = None

	def __init__(self):
		self.laserproj = LaserProjection()
		self.areaLeft = rospy.Publisher('areaLeft', PolygonStamped, queue_size=1)
		self.areaFront = rospy.Publisher('areaFront', PolygonStamped, queue_size=1)
		self.areaRight = rospy.Publisher('areaRight', PolygonStamped, queue_size=1)
		self.areaFRight = rospy.Publisher('areaFRight', PolygonStamped, queue_size=1)
		self.areaFLeft = rospy.Publisher('areaFLeft', PolygonStamped, queue_size=1)
		self.areaFHelp = rospy.Publisher('areaFHelp', PolygonStamped, queue_size=1)
		self.laserSub = rospy.Subscriber('/scan', LaserScan,queue_size = 1,callback=self.laserCallback)
		print "Wanderkennung gestartet"

	def calcAngles(self, numAngles,minAngle,angleIncrement):
		self.anglesCalculated = True
		cos_map = [np.cos(minAngle + i * angleIncrement) for i in range(numAngles)]
            	sin_map = [np.sin(minAngle + i * angleIncrement) for i in range(numAngles)]
		self.cos_sin_map = np.array([cos_map, sin_map])
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
		fha = self.frontHelpArea
		fleft = [Point32(x=fla.x1,y=fla.y1),Point32(x=fla.x1,y=fla.y2),Point32(x=fla.x2,y=fla.y2),Point32(x=fla.x2,y=fla.y1)]
		fright = [Point32(x=fra.x1,y=fra.y1),Point32(x=fra.x1,y=fra.y2),Point32(x=fra.x2,y=fra.y2),Point32(x=fra.x2,y=fra.y1)]
		fha = [Point32(x=fha.x1,y=fha.y1),Point32(x=fha.x1,y=fha.y2),Point32(x=fha.x2,y=fha.y2),Point32(x=fha.x2,y=fha.y1)]
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
		p.polygon.points=fha
		self.areaFHelp.publish(p)

	def laserCallback(self,data):
		if (rospy.is_shutdown() and not self.laserSub is None):
			self.laserSub.unregister()
			return
		if self.anglesCalculated == False:
			self.calcAngles(len(data.ranges),data.angle_min,data.angle_increment)
		startTime = time.time()
		ranges = np.array(data.ranges)
		ranges = np.array([ranges,ranges])
		output = ranges * self.cos_sin_map
		#counter fuer Felder
		inLeft  = 0
		inFront = 0
		inRight = 0
		inFrontLeft = 0
		inFrontRight = 0
		inFrontHelp = 0
		pIr = pointInRect
		x = 0
		y = 0
		#cloud = self.laserproj.projectLaser(data)
		#gen = pc2.read_points(cloud, skip_nans=True,field_names=("x","y"))
		n = len(data.ranges)
		for r in range(n):
			x = output[:,r][0]
			y = output[:,r][1]
			if (pIr(x,y,self.leftArea)):
				inLeft = inLeft + 1
			elif (pIr(x,y,self.frontArea)):
				inFront = inFront + 1
			elif (pIr(x,y,self.rightArea)):
				inRight = inRight + 1
			elif (pIr(x,y,self.frontRightArea)):
				inFrontRight = inFrontRight + 1
			elif (pIr(x,y,self.frontLeftArea)):
				inFrontLeft = inFrontLeft + 1
			elif (pIr(x,y,self.frontHelpArea)):
				inFrontHelp = inFrontHelp + 1
		left  = 'Frei' if inLeft  < 10 else 'Belegt'
		right = 'Frei' if inRight < 10 else 'Belegt'
		front = 'Frei' if inFront < 10 else 'Belegt'
		if inFrontHelp < 10 and (inFrontLeft > 10 or inFrontRight > 10):
			if (inFrontLeft > inFrontRight):
				self.wallToClose = 'links'
			else:
				self.wallToClose = 'rechts'
		else:
			self.wallToClose = ''
		self.lastWallInfo = self.WallInfo(left,front,right)
		endTime = time.time()
		#print (endTime-startTime)
		#print self.lastWallInfo, self.wallToClose

	def polar2Koord(self,angleIndex,range):
		x = range * self.cosAngles[angleIndex]
		y = range * self.sinAngles[angleIndex]
		return x,y

if __name__ == '__main__':
	rospy.init_node('WallDetection_MAIN')
	p = WallDetection()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Stopped")
