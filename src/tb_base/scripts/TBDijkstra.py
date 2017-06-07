def dijkstra(knoten, kanten, start, ziel):
	# knoten ist eine Liste von Knoten
	# kanten ist eine Liste von 3-Tupeln:
	#	(knoten1, knoten2, kosten)
	# start ist der Knoten, in dem die Suche startet
	# ziel ist der Knoten, zu dem ein Weg gesucht werden soll
	# Gibt ein Tupel zurück mit dem Weg und den Kosten 
	#
	knotenEigenschaften = [ [i, "inf", None, False] for i in knoten if i != start ]
	knotenEigenschaften += [ [start, 0, None, False] ]
	for i in range(len(knotenEigenschaften)):
		knotenEigenschaften[i] += [ i ]
 
	while True:
		unbesuchteKnoten = filter(lambda x: not x[3], knotenEigenschaften)
		if not unbesuchteKnoten: break
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
		return (weg, kosten)
	else:
		raise "Kein Weg gefunden"