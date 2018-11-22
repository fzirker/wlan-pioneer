#include "ros/ros.h"
#include "wlan_pioneer/WlanSignalMsg.h"
#include <sstream>

using namespace ros;
using namespace wlan_pioneer;

int main(int argc, char **argv)
{
  init(argc, argv, "wlanSignal");
  NodeHandle node;
  
  // topic: wlan_signal
  Publisher publisher = node.advertise<WlanSignalMsg>("wlan_signal", 1000);
  
  Rate loopRate(1); // every second

  while (ros::ok())
  {
    WlanSignalMsg msg;
    msg.timestamp = ros::Time::now();
    msg.level_24G = 0; // TODO: read real WLAN signal strength
    msg.level_5G = 0;

    ROS_INFO("Signal strength 2.4G: %li      Signal strength 5G: %li", msg.level_24G, msg.level_5G);

    publisher.publish(msg);
    spinOnce();
    loopRate.sleep();
  }
  return 0;
}
