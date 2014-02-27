#include "ros/ros.h"
#include <boost/bind.hpp>
#include "gaussian_process_regressor.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "wifi_mapping/wifi_measurement.h"
#include <map>

const std::string DEFAULT_WIFI_TOPIC = "/wifi_scanning_node/wifi_measurement";
const std::string DEFAULT_BASE_FRAME_ID = "base_footprint";
const std::string DEFAULT_FIXED_FRAME_ID = "map";

std::map<std::string,gaussian_process_regressor> gpr;

geometry_msgs::PoseWithCovarianceStamped current_pose;

std::string base_frame_id;
std::string fixed_frame_id;

std::shared_ptr<tf::TransformListener> pose_listener;
tf::StampedTransform pose_transform;

void pose_monitor(){
    // use tf to get the current pose estimate
    try {
        pose_listener->waitForTransform(fixed_frame_id,base_frame_id,ros::Time::now(),ros::Duration(1.0));
        pose_listener->lookupTransform(fixed_frame_id,base_frame_id,ros::Time(0), pose_transform);
    } catch (tf::TransformException ex){
        ROS_ERROR("Caught exception while looking up transform: %s",ex.what());
        return;
    }
}


void wifi_callback(ros::NodeHandle &nh, const wifi_mapping::wifi_measurement::ConstPtr &wifi_msg){
    // TODO this is done to ignore the access point running on our husky robot. Change to a parameter based setup (i.e. a yaml file with essid's to ignore)
    if (wifi_msg->essid.compare(0,3,"CPR")==0){
        return;
    }
    std::string ap_id = wifi_msg->id;
    //ROS_INFO("wifi_msg->header.seq: %d, wifi_msg->header.stamp: %f",wifi_msg->header.seq,wifi_msg->header.stamp.toSec());
    
    // create new GP if it does not already exist
    if (gpr.find(ap_id) == gpr.end() ){
        // keep a bounded number of GP in memory
        if(gpr.size() >=50){
            return;
        }
        gaussian_process_regressor gp_reg(nh, ap_id, fixed_frame_id);
        gpr[ap_id] = gp_reg;
    } 
    gpr[ap_id].process_measurement(pose_transform, wifi_msg);
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wifi_mapping_node");
    ros::NodeHandle nh("~");

    pose_listener = std::shared_ptr<tf::TransformListener>(new tf::TransformListener());

    std::string odom_topic, wifi_topic;
    nh.param<std::string>("wifi_topic",wifi_topic,DEFAULT_WIFI_TOPIC);
    nh.param<std::string>("fixed_frame_id",fixed_frame_id,DEFAULT_FIXED_FRAME_ID);
    nh.param<std::string>("base_frame_id",base_frame_id,DEFAULT_BASE_FRAME_ID);
  
    ros::Subscriber wifi_scan_sub = nh.subscribe<wifi_mapping::wifi_measurement>(wifi_topic, 1, boost::bind(&wifi_callback, boost::ref(nh), _1));

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::Rate r(50);
    while ( ros::ok() ){
        pose_monitor();        
        r.sleep();
    }
    return 0;
}
