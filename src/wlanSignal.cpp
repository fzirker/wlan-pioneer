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

int main(int argc, char **argv)
{
  // TODO: make interface configurable
  string singalLevelCmd("iwconfig wlp3s0 | grep Signal | cut -d \"=\" -f3 | cut -d \" \" -f1");

  init(argc, argv, "wlanSignal");
  NodeHandle node;

  // topic: wlan_signal
  Publisher publisher = node.advertise<WlanSignalMsg>("wlan_signal", 1000);

  Rate loopRate(1); // every second

  while (ros::ok())
  {
    WlanSignalMsg msg;
    msg.timestamp = ros::Time::now();
    msg.level_24G = stol(GetStdoutFromCommand(singalLevelCmd));
    msg.level_5G = 0;

    ROS_INFO("Signal strength 2.4G: %li      Signal strength 5G: %li", msg.level_24G, msg.level_5G);

    publisher.publish(msg);
    spinOnce();
    loopRate.sleep();
  }
  return 0;
}
