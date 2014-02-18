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
#include <mutex>

#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

const std::string DEFAULT_WIFI_TOPIC = "/wifi_scanning_node/wifi_measurement";
const std::string DEFAULT_ODOM_TOPIC = "/odom";

std::map<std::string,gaussian_process> GP;
std::map<std::string,double> variance_threshold;
std::map<std::string, std::function<void(void)>> gp_pcl_pub_funcs;
std::map<std::string, ros::Publisher> gp_pcl_pub_mean;
std::map<std::string, ros::Publisher> gp_pcl_pub_var;
std::map<std::string,std::shared_ptr<std::mutex>> gp_pcl_pub_mutex;

std::string published_gp_id = "";

void replace_(std::string &str){
  int i = str.find_first_of(":");
  while (i>=0)
    {
        str[i]='_';
        i=str.find_first_of(":",i+1);
    }
};

void create_new_pcl_publisher(std::string id){
    std::string id_new(id);
    replace_(id_new);
    ros::NodeHandle nh("~");
    ros::Publisher gp_pcl_pub_m = nh.advertise<sensor_msgs::PointCloud2>("pcloud_"+id_new+"_mean",1,true);
    ros::Publisher gp_pcl_pub_v = nh.advertise<sensor_msgs::PointCloud2>("pcloud_"+id_new+"_var",1,true);
    
    gp_pcl_pub_mutex[id] = std::shared_ptr<std::mutex>(new std::mutex());

    std::function<void(void)> gp_pcl_publish_clouds = [id,&gp_pcl_pub_mean,&gp_pcl_pub_var,&GP](){ 
        gp_pcl_pub_mutex[id]->lock();
        //if (gp_pcl_pub.getNumSubscribers()==0){
        //    return;
        //}
        ROS_INFO("Publishing point cloud for %s ",id.c_str());

        pcl::PointCloud<pcl::PointXYZI> cloud_mean, cloud_var;

        double grid_size = 50; // 10 meters
        int n_points = 30000; // 10 meters

        cloud_mean.header.frame_id = "odom";
        cloud_var.header.frame_id = "odom";

        int idx =0;
        double z_var;
        VectorXd z_mean;
        VectorXd X = VectorXd::Zero(GP[id].input_dimensions());

        for (idx=0; idx < n_points ; idx++){
                pcl::PointXYZI mean, variance;
                //cloud_val[idx].x = x;
                //cloud_val[idx].y = y;
                //cloud_var[idx].x = x;
                //cloud_var[idx].y = y;
                X = VectorXd::Random(2)*grid_size;
                mean.x = X[0]; variance.x = X[0];
                mean.y = X[1]; variance.y = X[1];

                // computed predicted singal measruement and variance
                //X[0]= x;
                //X[1]= y;
                GP[id].prediction(X,z_mean,z_var);

                //cloud_val[idx].z = z_val[0] ;
                //cloud_var[idx].z = z_var ;
                mean.z = z_mean[0];
                variance.z = z_var;
                mean.intensity = z_mean[0]; variance.intensity = z_var;
                cloud_mean.push_back(mean);
                cloud_var.push_back(variance);
        }
        
        // publish point cloud
        //pcl::toROSMsg(cloud_val,output);
        gp_pcl_pub_mean[id].publish(cloud_mean);
        gp_pcl_pub_var[id].publish(cloud_var);
        ROS_INFO("Published %d points ", idx);
        gp_pcl_pub_mutex[id]->unlock();
    };

    // create ros publisher object and publishing function pair. add it to the map
    gp_pcl_pub_mean[id] = gp_pcl_pub_m;
    gp_pcl_pub_var[id] = gp_pcl_pub_v;
    gp_pcl_pub_funcs[id] = gp_pcl_publish_clouds;
};

void gp_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_msg, const wifi_mapping::wifi_measurement::ConstPtr &wifi_msg){
    std::string ap_id = wifi_msg->id;
    geometry_msgs::Point pos = pose_msg->pose.pose.position;
    geometry_msgs::Quaternion quat = pose_msg->pose.pose.orientation;
    // construct measurement vector, for the moment, we only use location data
    VectorXd x(3);
    x << pos.x,pos.y,pos.z;
    double ss = wifi_msg->signal_strength/255.0;

    ROS_INFO("pose_msg time: %f, wifi_msg time: %f", pose_msg->header.stamp.toSec(), wifi_msg->header.stamp.toSec());
    ROS_INFO("Position [%f %f %f]",pos.x,pos.y,pos.z);
    ROS_INFO("Signal Strength [%f] ",ss*100.0);
    
    // create new GP if it does not already exist
    if (GP.find(ap_id) == GP.end() ){
        // keep a bounded number of GP in memory
        if(GP.size() >=5){
            return;
        }
        GP[ap_id] = gaussian_process();
        GP[ap_id].add_sample(x,ss);
        ROS_INFO("New sample for %s",ap_id.c_str());
        //ROS_INFO("Position [%f %f %f]",pos.x,pos.y,pos.z);
        //ROS_INFO("Orientation [%f %f %f %f]",quat.x,quat.y,quat.z,quat.w);
        //ROS_INFO("Signal Strength [%f] ",wifi_msg->signal_strength);
        GP[ap_id].set_SE_kernel();

        Vector4d init_params = VectorXd::Random(4).cwiseAbs()*4;
        for (int i=0; i<init_params.size(); i++){
            init_params(i) += 1.0;
        }
        GP[ap_id].set_opt_starting_point(init_params);
        create_new_pcl_publisher(ap_id);
        gp_pcl_pub_funcs[ap_id]();

        ROS_INFO("Created new GP estimator");
    } else {
        // TODO find a better way of adding samples
        double prediction_variance;
        VectorXd predicted_signal_strength;

        GP[ap_id].prediction(x,predicted_signal_strength,prediction_variance);
        //ROS_INFO("Measured Signal Strength: %f, Prediction: %f, Variance: %f",wifi_msg->signal_strength,predicted_signal_strength[0],prediction_variance);
        if(prediction_variance >= 0.5*variance_threshold[ap_id]){
            ROS_INFO("New sample for %s",ap_id.c_str());
            //ROS_INFO("Position [%f %f %f]",pos.x,pos.y,pos.z);
            //ROS_INFO("Orientation [%f %f %f %f]",quat.x,quat.y,quat.z,quat.w);
            //ROS_INFO("Signal Strength [%f] ",wifi_msg->signal_strength);

            gp_pcl_pub_mutex[ap_id]->lock();
            GP[ap_id].add_sample(x,ss);
            ROS_INFO("Total data points: %d",GP[ap_id].dataset_size());
            if(GP[ap_id].dataset_size()>10){
                GP[ap_id].optimize_parameters();
            }
            variance_threshold[ap_id] = GP[ap_id].compute_maximum_variance();
            gp_pcl_pub_mutex[ap_id]->unlock();
            gp_pcl_pub_funcs[ap_id]();
       }
    }
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "wifi_mapping_node");
    ros::NodeHandle nh("~");

    std::string odom_topic, wifi_topic;
    nh.param<std::string>("wifi_topic",wifi_topic,DEFAULT_WIFI_TOPIC);
    nh.param<std::string>("odometry_topic",odom_topic,DEFAULT_ODOM_TOPIC);
  
    message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> odom_sub(nh, odom_topic, 1);
    message_filters::Subscriber<wifi_mapping::wifi_measurement> wifi_scan_sub(nh, wifi_topic, 1);

    typedef message_filters::sync_policies::ApproximateTime<geometry_msgs::PoseWithCovarianceStamped, wifi_mapping::wifi_measurement> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, wifi_scan_sub);

    sync.registerCallback(boost::bind(&gp_callback, _1, _2));
    ros::spin();
    return 0;
}
