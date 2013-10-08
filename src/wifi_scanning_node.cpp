#include "wifi_scanner.h"
#include "ros/ros.h"
#include "wifi_mapping/wifi_measurement.h"

const std::string DEFAULT_IFACE = "wlan0";

ros::Publisher wifi_publisher;
ros::Subscriber pose_subscriber;

void process_scan(access_point &ap){
    wifi_mapping::wifi_measurement wm;

    wm.id = ap.mac_address;
    wm.channel = ap.frequency;
    wm.signal_strength = ap.signal_strength;
    wm.noise = ap.signal_noise;
    wm.essid = ap.essid;
    wm.header.stamp = ros::Time().fromSec(ap.timestamp);

    wifi_publisher.publish(wm);
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wifi_scanning_node");
    ros::NodeHandle n("~");
  
    wifi_publisher = n.advertise<wifi_mapping::wifi_measurement>("wifi_measurement", 1);
    //ros::Subscriber pose = n.subscribe(pose_topic, update_current);
    
    std::string interface_name;
    n.param<std::string>("interface_name",interface_name,DEFAULT_IFACE);
    wifi_scanner ws;
    
    std::function<scanning_callback> scan_callback = process_scan;
    ros::Rate r(1);

    if (ws.init(std::string(interface_name), scan_callback)){
        while ( ros::ok() ){
            ws.scan();
            r.sleep();
        }
    } else {
         ROS_ERROR("Could not open socket on %s", interface_name.c_str());
    }

    return 0;
}
