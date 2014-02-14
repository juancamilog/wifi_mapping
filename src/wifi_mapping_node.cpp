#include "ros/ros.h"
#include "gaussian_process.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "wifi_mapping/wifi_measurement.h"
#include <Eigen/Dense>
#include <map>

#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


std::map<std::string,gaussian_process> GP;
std::map<std::string,double> variance_threshold;
typedef std::pair<ros::Publisher,std::function<void(void)>> gp_pcl_pair;
std::map<std::string,gp_pcl_pair > gp_pcl_publishers;

std::string published_gp_id = "";

void replace_(std::string &str){
  unsigned int i = str.find_first_of(":");
  while (i!=std::string::npos)
    {
        str[i]='_';
        i=str.find_first_of(":",i+1);
    }
};

void create_new_pcl_publisher(std::string id){
    std::string id_new(id);
    replace_(id_new);
    ros::NodeHandle nh("~");
    ros::Publisher gp_pcl_pub = nh.advertise<sensor_msgs::PointCloud2>(id_new+"_pcl",1,true);
    
    std::function<void(void)> gp_pcl_publish_clouds = [id,&gp_pcl_publishers,&gp_pcl_pub,&GP](){ 
        if (gp_pcl_pub.getNumSubscribers()==0){
            return;
        }

        pcl::PointCloud<pcl::PointXYZ> cloud_val, cloud_var;
        sensor_msgs::PointCloud2 output;

        double grid_size = 50; // 50 meters
        double resolution = 0.5; //0.5 meters

        // cloud for storing the signal strength predictions
        cloud_val.width = std::floor(2.0*grid_size/resolution);
        cloud_val.height = 1;
        cloud_val.points.resize(cloud_val.width*cloud_val.height);
        // cloud for storing the variance of measurements
        cloud_var.width = std::floor(2.0*grid_size/resolution);
        cloud_var.height = 1;
        cloud_var.points.resize(cloud_var.width*cloud_var.height);

        int idx =0;
        double z_var;
        for (double x=-grid_size; x<grid_size; x+=resolution){
            for (double y=-grid_size; y<grid_size; y+=resolution){
                cloud_val[idx].x = x;
                cloud_val[idx].y = y;
                cloud_var[idx].x = x;
                cloud_var[idx].y = y;

                // computed predicted singal measruement and variance
                VectorXd X,z_val;
                X<< x,y;
                GP[id].prediction(X,z_val,z_var);

                cloud_val[idx].z = z_val[0] ;
                cloud_var[idx].z = z_var ;
            }
        }
        
        // publish point cloud
        gp_pcl_publishers[id].first.publish(output);
    };

    // create ros publisher object and publishing function pair. add it to the map
    gp_pcl_publishers[id] = gp_pcl_pair(gp_pcl_pub,gp_pcl_publish_clouds);

};

void gp_callback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg, wifi_mapping::wifi_measurement::ConstPtr &wifi_msg){
    std::string ap_id = wifi_msg->id;
    ROS_INFO("New data point for %s",ap_id.c_str());
    // create new GP if it does not already exist
    if (GP.find(ap_id) != GP.end() ){
        GP[ap_id] = gaussian_process();
        Vector4d init_params = VectorXd::Random(4).cwiseAbs()*10;
        GP[ap_id].set_opt_starting_point(init_params);
        create_new_pcl_publisher(ap_id);
    }

    geometry_msgs::Point pos = pose_msg->pose.pose.position;
    geometry_msgs::Quaternion quat = pose_msg->pose.pose.orientation;

    ROS_INFO("Position [%f %f %f]",pos.x,pos.y,pos.z);
    ROS_INFO("Orientation [%f %f %f %f]",quat.x,quat.y,quat.z,quat.w);
    ROS_INFO("Signal Strength [%f] ",wifi_msg->signal_strength);
    
    // construct measurement vector, for the moment, we only use location data
    VectorXd x(3);
    x << pos.x,pos.y,pos.z;

    // TODO find a better way of adding samples
    double prediction_variance;
    VectorXd predicted_signal_strength;

    GP[ap_id].prediction(x,predicted_signal_strength,prediction_variance);
    ROS_INFO("Measured Signal Strength: %f, Prediction: %f, Variance: %f",wifi_msg->signal_strength,predicted_signal_strength[0],prediction_variance);
    if(prediction_variance >= variance_threshold[ap_id]){
        ROS_INFO("Adding new measurement to GP set");
        GP[ap_id].add_sample(x,wifi_msg->signal_strength);
        GP[ap_id].optimize_parameters();
        variance_threshold[ap_id] = GP[ap_id].compute_maximum_variance();
        gp_pcl_publishers[ap_id].second();
   }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wifi_mapping_node");
    ros::NodeHandle nh("~");
  
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> odom_sub(nh, "/odom_combined", 1);
    message_filters::Subscriber<wifi_mapping::wifi_measurement> wifi_scan_sub(nh, "/wifi_scanning_node/wifi_measurement", 1);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, wifi_mapping::wifi_measurement> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, wifi_scan_sub);

    //sync.registerCallback(boost::bind(&gp_callback, _1, _2));
    ros::spin();
    return 0;
}
