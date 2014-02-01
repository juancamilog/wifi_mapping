#include "gaussian_process.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "wifi_mapping/wifi_measurement.h"
#include <Eigen/Dense>
#include <map>

std::map<Eigen::MatrixXd> position_data;
std::map<Eigen::MatrixXd> signal_measurements;

void gp_callback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & pose_msg, wifi_mapping::wifi_measurement::ConstPtr &wifi_msg){
    ROS_INFO("New data point for %s",wifi_msg.id.c_str());
    geometry_msgs::point pos = pose_msg.pose.pose.position;
    geometry_msgs::Quaternion quat = pose_msg.pose.pose.orientation;

    ROS_INFO("Position [%f %f %f]",pos.x,pos.y,pos.z);
    ROS_INFO("Orientation [%f %f %f %f]",quat.x,quat.y,quat.z,quat.w);
    ROS_INFO("Signal Strength [%f] ",wifi_msg.signal_strength);
    
    // construct measurement vector 
    MatrixXd x(3,1);
    
    // compute covariance of measurement vector with dataset
    
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wifi_mapping_node");
    ros::NodeHandle nh("~");
  
    ros::Subscriber pose = n.subscribe(pose_topic, update_current);
    
    message_filters::Subscriber<PoseWithCovarianceStamped> odom_sub(nh, "/odom_combined", 1);
    message_filters::Subscriber<wifi_measurement> wifi_scan_sub(nh, "/wifi_scanning_node/wifi_measurement", 1);

    typedef sync_policies::ApproximateTime<PoseWithCovarianceStamped, wifi_measurement> MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, wifi_scan_sub);

    sync.registerCallback(boost::bind(&gp_callback, _1, _2));
    ros::spin();
    return 0;
}
