#include "ros/ros.h"
#include "wlan_pioneer/WlanSignalMsg.h"
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <limits>
#include "wifi_scan.h"

using namespace std;
using namespace ros;
using namespace wlan_pioneer;

// Global variables for wifi scanning
const int MAX_APS = 64;            // the maximum amounts of APs (Access Points) we want to store
struct wifi_scan* wifi = nullptr;  // this stores all the library information
struct bss_info bss[MAX_APS];      // this is where we are going to keep informatoin about APs (Access Points)

WlanSignalMsg msg;

void scanInterface( const string& interface, const string& monitoringSsid ) {

   msg.level_2G4 = numeric_limits<int8_t>::min();
   msg.level_5G = numeric_limits<int8_t>::min();
   msg.cellCount_2G4 = 0;
   msg.cellCount_5G = 0;

   if ( wifi == nullptr ) {
      wifi = wifi_scan_init( interface.c_str() );
   }

   int status = wifi_scan_all( wifi, bss, MAX_APS );

   if ( status < 0 ) {
      ROS_ERROR( "Unable to get WLAN scan data: %s", std::strerror( errno ) );
      wifi_scan_close( wifi );
      wifi = nullptr;
      return;
   }
   else {
      for ( int i = 0; i < status && i < MAX_APS; ++i ) {
         if ( bss[i].seen_ms_ago > 100 ) {
            continue;  // we do not want old results
         }

         if ( std::strcmp( bss[i].ssid, monitoringSsid.c_str() ) != 0 ) {
            continue;
         }

         int8_t level = bss[i].signal_mbm / 100;
         if ( bss[i].frequency >= 2400 && bss[i].frequency <= 2499 ) {
            // this AP is sending on 2.4 GHz band
            msg.level_2G4 = std::max( msg.level_2G4, level );
            msg.cellCount_2G4++;
         }
         else if ( bss[i].frequency >= 5000 && bss[i].frequency <= 5999 ) {
            // this AP is sending on 5 GHz band
            msg.level_5G = std::max( msg.level_5G, level );
            msg.cellCount_5G++;
         }
      }
   }
}

int main( int argc, char** argv ) {
   init( argc, argv, "wlanSignal" );

   NodeHandle node;
   string wlanInterface;
   string monitoringSsid;

   if ( node.param<string>( "wlan_ssid", monitoringSsid, "" ) ) {
      ROS_INFO( "Param wlan_ssid: %s", monitoringSsid.c_str() );
   }
   else {
      ROS_ERROR( "Param 'wlan_ssid' not set. " );
   }

   if ( node.param<string>( "wlan_interface", wlanInterface, "wlan0" ) ) {
      ROS_INFO( "Param wlan_interface: %s", wlanInterface.c_str() );
   }
   else {
      ROS_INFO( "Param 'wlan_interface' not set. Using 'wlan0'" );
   }

   // topic: wlan_signal
   Publisher publisher = node.advertise<WlanSignalMsg>( "wlan_signal", 1000 );
   Rate loopRate( 1 );  // every second

   msg.ssid = monitoringSsid;

   while ( ok() ) {

      scanInterface( wlanInterface, monitoringSsid );
      msg.timestamp = Time::now();

      ROS_INFO( "Signal strength 2.4G: %i (%d Cells)   Signal strength 5G: %i (%d Cells)",
         msg.level_2G4,
         msg.cellCount_2G4,
         msg.level_5G,
         msg.cellCount_5G);

      publisher.publish( msg );
      spinOnce();
      loopRate.sleep();
   }

   wifi_scan_close( wifi );
   return 0;
}
