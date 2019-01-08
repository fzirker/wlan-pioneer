#!/usr/bin/python2

import csv
outFileName = 'visualisierung.csv'
file = open("wlan_pioneer_filtered.csv", "r")
csv_reader = csv.reader(file, delimiter="\n")
outFile = open(outFileName, "w")
for row in csv_reader:
	array=row[0].split(';')
	outFile.write("%s,%s,%s,%s\n"
         % (array[1], array[2], array[3], array[4]))
file.close()
outFile.close()

#Ergebnisdatei hat immer noch eine Überschrift. Manuell entfernen