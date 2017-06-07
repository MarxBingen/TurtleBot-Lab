#!/usr/bin/env python

def dijkstra(knoten, kanten, start, ziel):
	# knoten ist eine Liste von Knoten
	# kanten ist eine Liste von 3-Tupeln:
	#	(knoten1, knoten2, kosten)
	# start ist der Knoten, in dem die Suche startet
	# ziel ist der Knoten, zu dem ein Weg gesucht werden soll
	# Gibt ein Tupel zurueck mit dem Weg und den Kosten
	#
	knotenEigenschaften = [ [i, "inf", None, False] for i in knoten if i != start ]
	knotenEigenschaften += [ [start, 0, None, False] ]
	for i in range(len(knotenEigenschaften)):
		knotenEigenschaften[i] += [ i ]

	while True:
		unbesuchteKnoten = filter(lambda x: not x[3], knotenEigenschaften)
		if not unbesuchteKnoten:
			break
		sortierteListe = sorted(unbesuchteKnoten, key=lambda i: i[1])
		aktiverKnoten = sortierteListe[0]
		knotenEigenschaften[aktiverKnoten[4]][3] = True
		if aktiverKnoten[0] == ziel:
			break
		aktiveKanten = filter(lambda x: x[0] == aktiverKnoten[0], kanten)
		for kante in aktiveKanten:
			andererKnotenId = filter(lambda x: x[0] == kante[1], knotenEigenschaften)[0][4]
			gewichtSumme = aktiverKnoten[1]	+ kante[2]
			if gewichtSumme < knotenEigenschaften[andererKnotenId][1]:
				knotenEigenschaften[andererKnotenId][1] = gewichtSumme
				knotenEigenschaften[andererKnotenId][2] = aktiverKnoten[4]

	if aktiverKnoten[0] == ziel:
		weg = []
		weg += [ aktiverKnoten[0] ]
		kosten = 0
		while aktiverKnoten[0] != start:
			aktiverKnoten = knotenEigenschaften[aktiverKnoten[2]]
			weg += [ aktiverKnoten[0] ]
			kosten += aktiverKnoten[1]
		weg.reverse()
		return weg
	else:
		raise "Kein Weg gefunden"


if __name__== '__main__':
	knoten = []
	knoten.append("1,1")
	knoten.append("2,1")
	knoten.append("3,1")
	knoten.append("4,1")

	knoten.append("1,2")
	knoten.append("4,2")

	knoten.append("1,3")
	knoten.append("3,3")
	knoten.append("4,3")

	knoten.append("1,4")
	knoten.append("2,4")
	knoten.append("3,4")

	knoten.append("3,5")
	knoten.append("4,5")
	knoten.append("5,5")

	kanten = []
	kanten.append(("1,1","2,1",1))
	kanten.append(("1,1","1,2",1))

	kanten.append(("2,1","3,1",1))
	kanten.append(("3,1","4,1",1))

	kanten.append(("4,1","4,2",1))
	kanten.append(("4,2","4,3",1))
	kanten.append(("4,3","3,3",1))
	kanten.append(("3,3","3,4",1))
	kanten.append(("3,4","3,5",1))
	kanten.append(("3,5","4,5",1))
	kanten.append(("4,5","5,5",1))
	

	kanten.append(("1,2","1,3",1))
	kanten.append(("1,3","1,4",1))
	kanten.append(("1,4","2,4",1))
	kanten.append(("2,4","3,4",1))
	start = "1,1"
	ziel = "5,5"
	print dijkstra(knoten,kanten,start,ziel)
