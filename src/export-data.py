#!/usr/bin/python2

import rosbag
import sys
import os
from wlan_pioneer.msg import WlanSignalMsg
from geometry_msgs.msg import Point
import datetime

inFileName = os.path.abspath(sys.argv[1])
outFileName = os.path.join(os.path.dirname(inFileName), os.path.basename(inFileName).split(".")[0] + ".csv")
count = 0
pos = Point()
bag = rosbag.Bag(inFileName)
outFile = open(outFileName, "w")
outFile.write("SecsSinceEpoch;xPos;yPos;level2G4;level5G\n")

for topic, msg, timestamp in bag.read_messages(topics=['/wlan_signal', '/pose']):

   if (topic == "/pose"):
      pos = msg.pose.pose.position

   if (topic == "/wlan_signal"):
      timeStr = datetime.datetime.fromtimestamp(msg.timestamp.secs).isoformat()
      outFile.write("%s;%f;%f;%d;%d\n"
         % (timeStr, pos.x, pos.y, msg.level_2G4, msg.level_5G))

   count += 1

print "%d rows written to %s" % (count, outFileName)

outFile.close()
bag.close()
