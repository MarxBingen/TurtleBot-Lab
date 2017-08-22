#!/usr/bin/env python

import rospy
import math
import numpy as np
from threading import BoundedSemaphore
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PolygonStamped,Point32
from collections import namedtuple

from tb_base.msg import WallDetection

class TBWallDetection:

	laserSub = None
	lastScan = None
	#definition WallInfo
	WallInfo = namedtuple('WallInfo','links,mitte,rechts')
	lastWallInfo = WallInfo('Frei','Frei','Frei')
	#wand annaeherung links oder rechts
	wallToClose = ''
	#linkes feld
	FeldInfo = namedtuple('FeldInfo','x1,x2,y1,y2')
	frontArea = FeldInfo(0.0  , 0.20,-0.10, 0.10)
	nearFrontArea = FeldInfo(0.0 , 0.10,-0.10, 0.10)
	leftArea =  FeldInfo(-0.20,-0.05,-0.30,-0.18)
	rightArea = FeldInfo(-0.20,-0.05, 0.18, 0.30)

	frontHelpArea = FeldInfo(0.05,0.4,-0.05,0.05)
	frontLeftArea = FeldInfo(0.0,0.3,-0.18,-0.10)
	frontRightArea = FeldInfo(0.0,0.3,0.10,0.18)

	anglesCalculated = False
	cos_sin_map = None

	lock = BoundedSemaphore()

	def __init__(self):
		#self.areaLeft = rospy.Publisher('areaLeft', PolygonStamped, queue_size=1)
		#self.areaFront = rospy.Publisher('areaFront', PolygonStamped, queue_size=1)
		#self.areaRight = rospy.Publisher('areaRight', PolygonStamped, queue_size=1)
		#self.areaFRight = rospy.Publisher('areaFRight', PolygonStamped, queue_size=1)
		#self.areaFLeft = rospy.Publisher('areaFLeft', PolygonStamped, queue_size=1)
		#self.areaFHelp = rospy.Publisher('areaFHelp', PolygonStamped, queue_size=1)
		self.laserSub = rospy.Subscriber('/scan', LaserScan,queue_size = 1,callback=self.laserCallback)
		self.wallPub = rospy.Publisher('wallDetection',WallDetection,queue_size=1)
		print "Wanderkennung initialisiert"

	def calcAngles(self, numAngles,minAngle,angleIncrement):
		self.anglesCalculated = True
		cos_map = [np.cos(minAngle + i * angleIncrement) for i in range(numAngles)]
            	sin_map = [np.sin(minAngle + i * angleIncrement) for i in range(numAngles)]
		self.cos_sin_map = np.array([cos_map, sin_map])
		print "Sin/Cos-Tabellen erstellt"

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
		fhelp = [Point32(x=fha.x1,y=fha.y1),Point32(x=fha.x1,y=fha.y2),Point32(x=fha.x2,y=fha.y2),Point32(x=fha.x2,y=fha.y1)]
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
		p = PolygonStamped()
		p.header.frame_id="laser"
		p.header.stamp = rospy.Time.now()
		p.polygon.points=fhelp
		self.areaFHelp.publish(p)

	def laserCallback(self,data):
		if (rospy.is_shutdown() and not self.laserSub is None):
			self.laserSub.unregister()
			return
		if self.anglesCalculated == False:
			self.calcAngles(len(data.ranges),data.angle_min,data.angle_increment)
			#self.publishAreas()
		#zuerst entfernungen in ein numpy-array konvertieren
		ranges = np.array(data.ranges)
		#dann dieses verdoppeln, 2-dim
		ranges = np.array([ranges,ranges])
		#nun die werte mit den cos-sin werten je dim multiplizieren
		output = ranges * self.cos_sin_map
		#counter fuer Feldbelegungen
		inLeft  = 0
		inFront = 0
		inNearFront = 0
		inRight = 0
		inFrontLeft = 0
		inFrontRight = 0
		inFrontHelp = 0
		#ref auf Areas damit lesebarer, kuerzer
		lA = self.leftArea
		fA = self.frontArea
		nfA = self.nearFrontArea
		rA = self.rightArea
		flA = self.frontLeftArea
		frA = self.frontRightArea
		fhA = self.frontHelpArea
		#das output array drehen, damit x,y je index zusammengehoeren
		xys = np.rot90(output)
		#left
		ll = np.array([lA.x1,lA.y1])
		ur = np.array([lA.x2,lA.y2])
		drin = np.all(np.logical_and(ll <= xys, xys <= ur), axis=1)
		inLeft = np.sum(drin)
		#front
		ll = np.array([fA.x1,fA.y1])
		ur = np.array([fA.x2,fA.y2])
		drin= np.all(np.logical_and(ll <= xys, xys <= ur), axis=1)
		inFront = np.sum(drin)
		#nearfront
		ll = np.array([nfA.x1,nfA.y1])
		ur = np.array([nfA.x2,nfA.y2])
		drin= np.all(np.logical_and(ll <= xys, xys <= ur), axis=1)
		inNearFront = np.sum(drin)
		#rechts
		ll = np.array([rA.x1,rA.y1])
		ur = np.array([rA.x2,rA.y2])
		drin = np.all(np.logical_and(ll <= xys, xys <= ur), axis=1)
		inRight = np.sum(drin)
		#frontLinks
		ll = np.array([flA.x1,flA.y1])
		ur = np.array([flA.x2,flA.y2])
		drin = np.all(np.logical_and(ll <= xys, xys <= ur), axis=1)
		inFrontLeft = np.sum(drin)
		#frontRechts
		ll = np.array([frA.x1,frA.y1])
		ur = np.array([frA.x2,frA.y2])
		drin = np.all(np.logical_and(ll <= xys, xys <= ur), axis=1)
		inFrontRight = np.sum(drin)
		#frontHilfe
		ll = np.array([fhA.x1,fhA.y1])
		ur = np.array([fhA.x2,fhA.y2])
		drin = np.all(np.logical_and(ll <= xys, xys <= ur), axis=1)
		inFrontHelp = np.sum(drin)
		#Auswertung False = Frei nach WallDetection.msg
		wd = WallDetection()
		wd.left  = False if inLeft  < 20 else True
		wd.right = False if inRight < 20 else True
		wd.front = False if inFront < 50 else True
		#Close 0 = frei 1=links 2=mitte 3=rechts
		if (inFrontHelp < 10) and (inFrontLeft > 10 or inFrontRight > 10):
			if (inFrontLeft > inFrontRight):
				wd.close = 1
			else:
				wd.close = 3
		else:
			wd.close = 0
		#ueberschreibt annaeherung wenn zu nah an wand
		if inNearFront > 20:
			wd.close = 2
		self.wallPub.publish(wd)

if __name__ == '__main__':
	print "Wanderkennung gestartet"
	rospy.init_node('WallDetection')
	p = TBWallDetection()
	rospy.spin()
