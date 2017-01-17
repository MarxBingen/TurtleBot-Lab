#!/usr/bin/env python

from enum import Enum

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
