#include "ros/ros.h"
#include "wlan_pioneer/WlanSignalMsg.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;
using namespace ros;
using namespace wlan_pioneer;

string GetStdoutFromCommand(const string &cmd)
{
   string data;
   FILE *stream;
   const int max_buffer = 256;
   char buffer[max_buffer];
   //cmd.append(" 2>&1");

   stream = popen(cmd.c_str(), "r");
   if (stream)
   {
      while (!feof(stream))
      {
         if (fgets(buffer, max_buffer, stream) != NULL)
         {
            data.append(buffer);
         }
      }
      pclose(stream);
   }
   return data;
}

int64_t getWlanSignalStrength(const string &interface)
{
   stringstream command;
   command << "iwconfig " << interface << " | grep Signal "
           << "| cut -d \"=\" -f3 "
           << "| cut -d \" \" -f1";
   return stol(GetStdoutFromCommand(command.str()));
}

int main(int argc, char **argv)
{
   init(argc, argv, "wlanSignal");
   NodeHandle node;
   string wlanInterface24G;
   string wlanInterface5G;

   if (node.param<string>("wlan_interface_24G", wlanInterface24G, "wlan0"))
   {
      ROS_INFO("Param wlan_interface_24: %s", wlanInterface24G.c_str());
   }
   else
   {
      ROS_INFO("Param 'wlan_interface_24' not set. Using 'wlan0'");
   }

   if (node.param<string>("wlan_interface_5G", wlanInterface5G, "wlan0"))
   {
      ROS_INFO("Param wlan_interface_5G: %s", wlanInterface5G.c_str());
   }
   else
   {
      ROS_INFO("Param 'wlan_interface_5G' not set. Using 'wlan0'");
   }

   // topic: wlan_signal
   Publisher publisher = node.advertise<WlanSignalMsg>("wlan_signal", 1000);

   Rate loopRate(1); // every second

   while (ros::ok())
   {
      WlanSignalMsg msg;
      msg.timestamp = ros::Time::now();
      msg.level_24G = getWlanSignalStrength(wlanInterface24G);
      msg.level_5G = getWlanSignalStrength(wlanInterface5G);

      ROS_INFO("Signal strength 2.4G: %li   Signal strength 5G: %li", msg.level_24G, msg.level_5G);

      publisher.publish(msg);
      spinOnce();
      loopRate.sleep();
   }
   return 0;
}
