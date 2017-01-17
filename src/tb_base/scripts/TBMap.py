#!/usr/bin/env python

import rospy
from TBHeading import SimpleHeading

class TBMap:
	
	mapArray = []
	posX=0
	posY=0
	heading = SimpleHeading.NORD

	def __init__(self,size):
		self.size=size
		mapArray = [[-1]*size*2]*size*2
		#Ausgangsposition auf null setzen
		mapArray[size][size]=0
		self.posX = size
		self.posY = size

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

