#include "signal_strength_estimator.h"
signal_strength_estimator::signal_strength_estimator(){
    gp_mutex = std::shared_ptr<std::mutex>(new std::mutex());
    GP = new gaussian_process(3);
    GP->set_opt_random_start(5,5);
}

signal_strength_estimator::signal_strength_estimator(ros::NodeHandle &nh, std::string id, std::string frame_id){

    ap_id = std::string(id);
    fixed_frame_id = frame_id;

    gp_mutex = std::shared_ptr<std::mutex>(new std::mutex());

    int i = id.find_first_of(":");
    while (i>=0) {
        ap_id[i]='_';
        i=ap_id.find_first_of(":",i+1);
    }

    mean_publisher = nh.advertise<sensor_msgs::PointCloud2>("pcloud_"+ap_id+"_mean",1,true);
    variance_publisher = nh.advertise<sensor_msgs::PointCloud2>("pcloud_"+ap_id+"_var",1,true);

    GP = new gaussian_process(3);
    GP->set_opt_random_start(5,5);
}

void signal_strength_estimator::publish_clouds(){ 
    /*if (mean_publisher.getNumSubscribers()==0 && variance_publisher.getNumSubscribers()==0){
        gp_mutex->unlock();
        return;
    }*/
    ROS_INFO("Publishing point cloud for %s ",ap_id.c_str());

    pcl::PointCloud<pcl::PointXYZI> cloud_mean, cloud_var;

    double grid_size = 15; // 10 meters
    int n_points = 50000; 

    cloud_mean.header.frame_id = fixed_frame_id;
    cloud_var.header.frame_id = fixed_frame_id;

    int idx =0;
    double z_var;
    Eigen::VectorXd z_mean;
    Eigen::VectorXd X = VectorXd::Zero(GP->input_dimensions());

    for (idx=0; idx < n_points ; idx++){
            
            pcl::PointXYZI mean, variance;
            X = VectorXd::Random(3)*grid_size;
            mean.x = X[0]; variance.x = X[0];
            mean.y = X[1]; variance.y = X[1];
            X(2) = 0.0;

            // computed predicted singal measruement and variance
            GP->prediction(X,z_mean,z_var);

            mean.z = z_mean[0];
            variance.z = z_var;
            mean.intensity = z_mean[0]; variance.intensity = z_var;
            cloud_mean.push_back(mean);
            cloud_var.push_back(variance);
    }
    // publish point cloud
    mean_publisher.publish(cloud_mean);
    variance_publisher.publish(cloud_var);
    ROS_INFO("Published %d points ", idx);
};

void signal_strength_estimator::process_measurement(tf::StampedTransform &pose_transform, const wifi_mapping::wifi_measurement::ConstPtr &wifi_msg){
    gp_mutex->lock();
    // construct measurement vector, for the moment, we only use translation data
    Eigen::Vector3d pos_x;
    tf::vectorTFToEigen(pose_transform.getOrigin(),pos_x);
    Eigen::VectorXd x(pos_x);

    double ss = wifi_msg->signal_strength; 
     
    ROS_INFO("New sample for %s (essid: %s)",ap_id.c_str(),wifi_msg->essid.c_str()); 
    ROS_INFO("pose_msg time: %f, wifi_msg time: %f", pose_transform.stamp_.toSec(), wifi_msg->header.stamp.toSec()); 
    ROS_INFO("Position [%f %f %f]",x[0],x[1],x[2]); 
    ROS_INFO("Signal Strength [%f] ",ss); 


    // TODO find a better way of adding samples
    double prediction_variance;
    Eigen::VectorXd predicted_signal_strength;

    GP->prediction(x,predicted_signal_strength,prediction_variance);
    variance_threshold = GP->compute_maximum_variance();
    ROS_INFO("Measured Signal Strength: %f, Prediction: %f, Variance: %f, Variance Threshold: %f",wifi_msg->signal_strength,predicted_signal_strength[0],prediction_variance, variance_threshold);
    bool add_measurement = (prediction_variance >= variance_threshold) || (GP->dataset_size()<2);
    
    if(add_measurement){
        ROS_INFO("Adding sample to GP estimator");
        GP->add_sample(x,ss);

        ROS_INFO("Added sample to GP estimator");
        ROS_INFO("Total data points: %d",GP->dataset_size());
        // optimize every 20 data points
        /*if(GP->dataset_size()%20 == 0 && GP->dataset_size() > 2){
            ROS_INFO_STREAM("Optimizing "<<ap_id);
            gp_mutex->lock();
            ROS_INFO("mutex locked ");
            GP->init(GP->kernel->parameters);
            ROS_INFO("init gp ");
            GP->set_opt_random_start(100,100);
            GP->optimize_parameters();
            ROS_INFO_STREAM("new log likelihood "<<GP->log_marginal_likelihood());
            gp_mutex->unlock();
            ROS_INFO("mutex unlocked ");
        }*/
   }
   publish_clouds();
   gp_mutex->unlock();
};

void signal_strength_estimator::optimize(double stopping_criterion, int solver, int restarts, double scale, double offset){
    if(GP->dataset_size() > 5){
        gp_mutex->lock();
        GP->init(GP->kernel->parameters);
        GP->optimize_parameters_random_restarts(stopping_criterion, solver, restarts, scale, offset);
        gp_mutex->unlock();
        //gp_mutex->lock();
        //GP->init(GP->kernel->parameters);
        //GP->set_opt_random_start(1000,1000);
        //GP->optimize_parameters();
        //ROS_INFO_STREAM("new log likelihood "<<GP->log_marginal_likelihood());
    }
};
