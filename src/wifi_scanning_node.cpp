#include <chrono>
#include <thread>
#include <algorithm>
#include "wifi_scanner.h"
#include "ros/ros.h"
#include "wifi_mapping/wifi_measurement.h"
#include "wifi_mapping/get_wifi_signature.h"

const std::string DEFAULT_IFACE = "wlan0";
const double DEFAULT_RATE_HZ = 1.0;

ros::Publisher wifi_publisher;
ros::Subscriber pose_subscriber;
std::map<std::string,wifi_mapping::wifi_measurement> wifi_signature;

bool compute_signature = false;
double alpha = 0.75;

struct {
    bool operator()(wifi_mapping::wifi_measurement a, wifi_mapping::wifi_measurement b){
        return a.signal_strength > b.signal_strength;
    }
} signal_strength_compare;

void process_scan(access_point &ap){
    wifi_mapping::wifi_measurement wm;

    wm.id = ap.mac_address;
    wm.channel = ap.frequency;
    wm.signal_strength = ap.signal_strength;
    wm.noise = ap.signal_noise;
    wm.essid = ap.essid;
    wm.header.stamp = ros::Time().fromSec(ap.timestamp);

    wifi_publisher.publish(wm);
    //ROS_INFO("wifi_scanning_node:: published %s",wm.id.c_str());

    if(compute_signature){
        //ROS_INFO("wifi_scanning_node:: computing signature");
        if ( wifi_signature.find(wm.id) == wifi_signature.end() ){
            wifi_signature[wm.id] = wm;
            ROS_INFO("wifi_scanning_node:: Found new access point %s",wifi_signature[wm.id].id.c_str());
        } else { 
            wifi_signature[wm.id].header.seq += 1;
            double delta = wm.signal_strength - wifi_signature[wm.id].signal_strength;
            wifi_signature[wm.id].signal_strength = wifi_signature[wm.id].signal_strength + delta/wifi_signature[wm.id].header.seq;
            delta = wm.noise - wifi_signature[wm.id].noise;
            wifi_signature[wm.id].noise = wifi_signature[wm.id].noise + delta/wifi_signature[wm.id].header.seq;
            wifi_signature[wm.id].header.stamp = wm.header.stamp;
            ROS_INFO("wifi_scanning_node:: Updated %s with signal strength %f",wifi_signature[wm.id].id.c_str(),wifi_signature[wm.id].signal_strength);
        }
    }
};

bool get_wifi_signature(wifi_mapping::get_wifi_signature::Request &req, wifi_mapping::get_wifi_signature::Response &res){
    ROS_INFO("wifi_scanning_node:: Got call to compute the wifi signature with arguments [%d %f]",req.size,req.timeout);
    double timeout = req.timeout;
    std::chrono::duration<double> elapsed_time(0.0);
    std::chrono::duration<double> sleep_duration(timeout/10.0);
    std::chrono::time_point<std::chrono::system_clock> start,end; 
    wifi_signature.clear();
    compute_signature = true;
    start = std::chrono::system_clock::now();
    while(elapsed_time.count() < timeout){
         ROS_INFO("wifi_scanning_node:: Computing the wifi signature. %f seconds left...", (timeout - elapsed_time.count()));
         std::this_thread::sleep_for(sleep_duration);
         end = std::chrono::system_clock::now();
         elapsed_time = end-start;
    }
    ROS_INFO("wifi_scanning_node:: Got %d access points.", (int)wifi_signature.size());
    compute_signature = false;
    // Convert the wifi_signature map into two vectors
    std::vector<wifi_mapping::wifi_measurement> signature_vector;
    for ( std::map<std::string,wifi_mapping::wifi_measurement>::const_iterator it = wifi_signature.begin(); it != wifi_signature.end(); ++it){
        signature_vector.push_back( it->second );
    }

    // sort and get the k strongest signals, where k is defined in the service request
    std::sort(signature_vector.begin(),signature_vector.end(),signal_strength_compare);
    int len = signature_vector.size() < req.size? signature_vector.size():req.size;

    std::vector<wifi_mapping::wifi_measurement> signature_v(signature_vector.begin(),signature_vector.begin()+len);
    res.signature = signature_v;
    return true;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wifi_scanning_node");
    ros::NodeHandle n("~");
  
    wifi_publisher = n.advertise<wifi_mapping::wifi_measurement>("wifi_measurement", 1);
    //ros::Subscriber pose = n.subscribe(pose_topic, update_current);
    
    std::string interface_name;
    n.param<std::string>("interface_name",interface_name,DEFAULT_IFACE);

    double scanning_rate_hz;
    n.param<double>("scanning_rate",scanning_rate_hz,DEFAULT_RATE_HZ);

    ROS_INFO("wifi_scanning_node:: Scanning on %s at %f hz.", interface_name.c_str(), scanning_rate_hz);
    wifi_scanner ws;
    
    ros::ServiceServer signature_service = n.advertiseService("get_wifi_signature",get_wifi_signature);

    std::function<scanning_callback> scan_callback = process_scan;
    ros::Rate r(scanning_rate_hz);
    ros::AsyncSpinner spinner(1);
    spinner.start();
    if (ws.init(std::string(interface_name), scan_callback)){
        while ( ros::ok() ){
            int res = ws.scan();
            if (res!=0){
                 ROS_ERROR("wifi_scanning_node:: Scan failed. Return code was %d", res);
            }
            //ros::spinOnce();
            r.sleep();
        }
    } else {
         ROS_ERROR("wifi_scanning_node:: Could not open socket on %s", interface_name.c_str());
    }
    spinner.stop();

    return 0;
}
