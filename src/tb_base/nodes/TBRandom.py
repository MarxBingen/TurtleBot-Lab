#!/usr/bin/env python

from random import randint
#gibt einen zufaelligen Wert aus der Liste zurueck
def zufall(valueList):
	if (len(valueList)==0):
		return 'sackgasse'
	r = randint(0,len(valueList)-1)
	return valueList[r]

def zufallBelegung(belegung):
	v = []
	if p.rechts == 'Frei':
		v.append('rechts')
	if p.links == 'Frei':
		v.append('links')
	if p.mitte == 'Frei':
		v.append('mitte')
	if len(v) == 0:
		return 'sackgasse'
	else:
		return zufall(v)
