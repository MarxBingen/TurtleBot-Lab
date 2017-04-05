#!/usr/bin/env python

from enum import Enum
import math

class SimpleHeading(Enum):
	NORD = 1
	OST = 2
	SUED = 3
	WEST = 4 

	def turn(oldHeading,richtung):
		nv = oldHeading.value - 1 if richtung=='links' else oldHeading.value+1
		if (nv < 1):
			nv = 4
		if (nv > 4):
			nv = 1
		return SimpleHeading(nv)

	def yaw(heading):
		y = 0
		if (heading is SimpleHeading.NORD):
			y = 0
		elif heading is SimpleHeading.WEST:
			y=4.71238898
		elif heading is SimpleHeading.SUED:
			y=3.14159265
		elif heading is SimpleHeading.OST:
			y=1.57079633
		return y
