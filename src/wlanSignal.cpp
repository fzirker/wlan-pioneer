#include "ros/ros.h"
#include "wlan_pioneer/WlanSignalMsg.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <linux/wireless.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <cstring>
#include <limits>

using namespace std;
using namespace ros;
using namespace wlan_pioneer;


int8_t getWlanSignalStrength(const string &interface) {
   struct iw_statistics stats;
   struct iwreq req;
   memset(&stats, 0, sizeof(stats));
   memset(&req, 0, sizeof(iwreq));
   strncpy(req.ifr_name, interface.c_str(), 16);
   req.u.data.pointer = &stats;
   req.u.data.length = sizeof(iw_statistics);
#ifdef CLEAR_UPDATED
   req.u.data.flags = 1;
#endif

   int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
   if(sockfd == -1) {
      ROS_ERROR("Could not create simple datagram socket");
      return numeric_limits<int8_t>::min() ;
   }
   if(ioctl(sockfd, SIOCGIWSTATS, &req) == -1) {
      ROS_ERROR("Error retrieving WLAN signal strength ");
      close(sockfd);
      return numeric_limits<int8_t>::min() ;

   }
   close(sockfd);
   return stats.qual.level;
}


int main(int argc, char **argv) {
   init(argc, argv, "wlanSignal");
   NodeHandle node;
   string wlanInterface24G;
   string wlanInterface5G;

   if (node.param<string>("wlan_interface_24G", wlanInterface24G, "wlan0")) {
      ROS_INFO("Param wlan_interface_24G: %s", wlanInterface24G.c_str());
   }
   else {
      ROS_INFO("Param 'wlan_interface_24G' not set. Using 'wlan0'");
   }

   if (node.param<string>("wlan_interface_5G", wlanInterface5G, "wlan0")) {
      ROS_INFO("Param wlan_interface_5G: %s", wlanInterface5G.c_str());
   }
   else {
      ROS_INFO("Param 'wlan_interface_5G' not set. Using 'wlan0'");
   }

   // topic: wlan_signal
   Publisher publisher = node.advertise<WlanSignalMsg>("wlan_signal", 1000);
   Rate loopRate(1); // every second

   while (ok()) {
      WlanSignalMsg msg;
      msg.timestamp = Time::now();
      msg.level_24G = getWlanSignalStrength(wlanInterface24G);
      msg.level_5G = getWlanSignalStrength(wlanInterface5G);

      ROS_INFO("Signal strength 2.4G: %i   Signal strength 5G: %i", msg.level_24G, msg.level_5G);

      publisher.publish(msg);
      spinOnce();
      loopRate.sleep();
   }
   return 0;
}
