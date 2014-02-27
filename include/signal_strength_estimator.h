#include "ros/ros.h"
#include "gaussian_process.h"
#include <Eigen/Dense>
#include <mutex>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "wifi_mapping/wifi_measurement.h"

class signal_strength_estimator{
    private:
        gaussian_process* GP;
        double variance_threshold;
        std::string fixed_frame_id;
        std::string ap_id;
        std::string essid;
        ros::Publisher mean_publisher;
        ros::Publisher variance_publisher;

        std::shared_ptr<std::mutex> gp_mutex;

    public:
        signal_strength_estimator();
        signal_strength_estimator(ros::NodeHandle &nh, std::string id, std::string frame_id);
        void publish_clouds();
        void process_measurement(tf::StampedTransform &pose_transform, const wifi_mapping::wifi_measurement::ConstPtr &wifi_msg);
};
