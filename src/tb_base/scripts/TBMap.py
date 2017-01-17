#!/usr/bin/env python

import rospy
from TBHeading import SimpleHeading
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose,Point
class TBMap:
	
	mapArray = []
	posX=0
	posY=0
	heading = SimpleHeading.NORD
	map = OccupancyGrid()
	mapPub = None

	def __init__(self,size,raster):
		self.size=size
		self.raster=raster
		self.mapArray = [-1]*(size*2*size*2)
		print self.mapArray
		#Ausgangsposition auf null setzen
		self.posX = size
		self.posY = size
		self.mapPub = rospy.Publisher('map',OccupancyGrid,queue_size=1)

	def turn(self,richtung):
		self.heading = SimpleHeading.turn(self.heading,richtung)
		print self.heading

	def printPosition(self):
		print "Position:", self.posX-self.size, self.posY-self.size
		print "Heading:",self.heading

	def updatePosition(self):
		if self.heading is SimpleHeading.NORD:
			self.posY = self.posY + 1
		elif self.heading is SimpleHeading.SUED:
			self.posY = self.posY - 1
		elif self.heading is SimpleHeading.WEST:
			self.posX = self.posX - 1
		elif self.heading is SimpleHeading.OST:
			self.posX = self.posX + 1

	def updateMap(self, feldbelegung):
		posX=self.posX
		posY=self.posY
		s=self.size
		print self.mapArray
		self.mapArray[posY*self.size+posX]=0
		if self.heading is SimpleHeading.NORD:
			self.mapArray[posY+1 *s+ posX  ] = 0 if feldbelegung.mitte == 'Frei' else 100
			self.mapArray[posY   *s+ posX-1] = 0 if feldbelegung.links == 'Frei' else 100
			self.mapArray[posY   *s+ posX+1] = 0 if feldbelegung.rechts == 'Frei' else 100
		elif self.heading is SimpleHeading.SUED:
			self.mapArray[posY-1 *s+ posX  ]= 0 if feldbelegung.mitte == 'Frei' else 100
			self.mapArray[posY   *s+ posX+1]= 0 if feldbelegung.links == 'Frei' else 100
			self.mapArray[posY   *s+ posX-1]= 0 if feldbelegung.rechts == 'Frei' else 100
		elif self.heading is SimpleHeading.WEST:
			self.mapArray[posY   *s+ posX-1]= 0 if feldbelegung.mitte == 'Frei' else 100
			self.mapArray[posY-1 *s+ posX  ]= 0 if feldbelegung.links == 'Frei' else 100
			self.mapArray[posY+1 *s+ posX  ]= 0 if feldbelegung.rechts == 'Frei' else 100
		elif self.heading is SimpleHeading.OST:
			self.mapArray[posY   *s+ posX+1]= 0 if feldbelegung.mitte == 'Frei' else 100
			self.mapArray[posY+1 *s+ posX  ]= 0 if feldbelegung.links == 'Frei' else 100
			self.mapArray[posY-1 *s+ posX  ]= 0 if feldbelegung.rechts == 'Frei' else 100
		#self.updateOccupancyGrid()

	def updateOccupancyGrid(self):
		self.map.info.resolution=self.raster
		self.map.info.width=self.size*2
		self.map.info.height=self.size*2
		self.map.info.origin=Pose()
		self.map.info.origin.position=Point(-self.size*self.raster,-self.size*self.raster,0)
		self.map.header.frame_id = "map"
		self.map.header.stamp = rospy.Time.now()
		self.map.data=self.mapArray
		self.mapPub.publish(self.map) 
