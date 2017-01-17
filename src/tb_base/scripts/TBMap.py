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
		self.mapArray = [[int(-1)]*size*2]*size*2
		#Ausgangsposition auf null setzen
		self.mapArray[size][size]=0
		self.posX = size
		self.posY = size
		self.mapPub = rospy.Publisher('map',OccupancyGrid,queue_size=1)

	def turn(self,richtung):
		self.heading = SimpleHeading.turn(self.heading,richtung)

	def printPosition(self):
		print "Position:", self.posX-self.size, self.posY-self.size

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
		self.mapArray[posX][posY]=0
		if self.heading is SimpleHeading.NORD:
			self.mapArray[posX][posY+1]= 0 if feldbelegung.mitte == 'Frei' else 100
			self.mapArray[posX-1][posY]= 0 if feldbelegung.links == 'Frei' else 100
			self.mapArray[posX+1][posY]= 0 if feldbelegung.rechts == 'Frei' else 100
		elif self.heading is SimpleHeading.SUED:
			self.mapArray[posX][posY-1]= 0 if feldbelegung.mitte == 'Frei' else 100
			self.mapArray[posX+1][posY]= 0 if feldbelegung.links == 'Frei' else 100
			self.mapArray[posX-1][posY]= 0 if feldbelegung.rechts == 'Frei' else 100
		elif self.heading is SimpleHeading.WEST:
			self.mapArray[posX-1][posY]= 0 if feldbelegung.mitte == 'Frei' else 100
			self.mapArray[posX][posY-1]= 0 if feldbelegung.links == 'Frei' else 100
			self.mapArray[pos1][posY+1]= 0 if feldbelegung.rechts == 'Frei' else 100
		elif self.heading is SimpleHeading.OST:
			self.mapArray[posX+1][posY]= 0 if feldbelegung.mitte == 'Frei' else 100
			self.mapArray[posX][posY+1]= 0 if feldbelegung.links == 'Frei' else 100
			self.mapArray[posX][posY-1]= 0 if feldbelegung.rechts == 'Frei' else 100
		self.updateOccupancyGrid()

	def updateOccupancyGrid(self):
		self.map.info.resolution=self.raster
		self.map.info.width=self.size*2
		self.map.info.height=self.size*2
		self.map.info.origin=Pose()
		self.map.info.origin.position=Point(-self.size,-self.size,0)
		self.map.header.frame_id = "map"
		self.map.header.stamp = rospy.Time.now()
		self.map.data=[]
		for lines in range(0,len(self.mapArray)):
			self.map.data.extend(self.mapArray[lines])
		print len(self.map.data)
		self.mapPub.publish(self.map) 
