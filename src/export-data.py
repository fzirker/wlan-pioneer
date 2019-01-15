#!/usr/bin/python2

import rosbag
import sys
import os
from wlan_pioneer.msg import WlanSignalMsg
import datetime
import tf2_msgs
import geometry_msgs
import rospy
import tf2_ros


rospy.init_node('wlan_bag_listener')

inFileName = os.path.abspath(sys.argv[1])
outFileName = os.path.join(os.path.dirname(inFileName), os.path.basename(inFileName).split(".")[0] + ".csv")
countOut = 0
outFile = open(outFileName, "w", buffering=1)
trans = geometry_msgs.msg.TransformStamped()
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)
rate = rospy.Rate(10.0)

def wlanCallback(msg, args):
      trans = geometry_msgs.msg.TransformStamped()
      outFile = args[0]
      if (msg.level_2G4 == -128 and msg.level_5G == -128):
         return

      try:
         trans = tfBuffer.lookup_transform("map", "base_link", rospy.Time())
      except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
         pass

      rospy.loginfo ("line %d: %s,%f,%f,%d,%d" %
         (0, msg.timestamp.secs, trans.transform.translation.x, trans.transform.translation.y,
         msg.level_2G4, msg.level_5G))

      outFile.write("%s,%f,%f,%d,%d\n"
         % (msg.timestamp.secs, trans.transform.translation.x, trans.transform.translation.y,
         msg.level_2G4, msg.level_5G))


rospy.Subscriber("wlan_signal", WlanSignalMsg, wlanCallback, (outFile,))
outFile.write("SecsSinceEpoch,xPos,yPos,level2G4,level5G\n")

rospy.spin()

#rospy.loginfo "%d Messages read from bag file\n%d rows written to %s" % (countBag, countOut, outFileName)

outFile.close()


