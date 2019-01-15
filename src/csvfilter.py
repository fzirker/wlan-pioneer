#!/usr/bin/python2

import csv
outFileName = 'visualisierung1712.csv'
file = open("wlan_pioneer_2018-12-17-17-14-03.csv", "r")
csv_reader = csv.reader(file, delimiter="\n")
outFile = open(outFileName, "w")
begin = True
for row in csv_reader:
	if begin == False:
		array=row[0].split(',')
		g24 = (-1 * float(array[3]) - 40)/40
		g5 = (-1 * float(array[4]) - 40)/40
		outFile.write("%s,%s,%f,%f\n"
			% (array[1], array[2], g24, g5))
	begin = False
file.close()
outFile.close()

#Signalstaerken sind zwischen 0 und 1 normalisiert.