#!/usr/bin/env python

from random import randint
#gibt einen zufaelligen Wert aus der Liste zurueck
def zufall(valueList):
	r = randint(0,len(valueList)-1)
	return valueList[r]