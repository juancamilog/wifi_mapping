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
}

void signal_strength_estimator::publish_clouds(){ 
    /*if (mean_publisher.getNumSubscribers()==0 && variance_publisher.getNumSubscribers()==0){
        gp_mutex->unlock();
        return;
    }*/
    //ROS_INFO("Publishing point cloud for %s ",ap_id.c_str());

    pcl::PointCloud<pcl::PointXYZI> cloud_mean, cloud_var;

    double grid_size = 15; // 10 meters
    int n_points = 50000; 

    cloud_mean.header.frame_id = fixed_frame_id;
    cloud_var.header.frame_id = fixed_frame_id;

    int idx =0;
    double z_var;
    Eigen::VectorXd z_mean;
    Eigen::VectorXd X = VectorXd::Zero(GP->input_dimensions());

    double var_norm = 1.0/variance_threshold;

    for (idx=0; idx < n_points ; idx++){
            pcl::PointXYZI mean, variance;
            X = VectorXd::Random(3)*grid_size;
            mean.x = X[0]; variance.x = X[0];
            mean.y = X[1]; variance.y = X[1];
            X(2) = 0.0;

            // computed predicted singal measruement and variance
            GP->prediction(X,z_mean,z_var);
            if (std::fabs(z_var - variance_threshold)<1e-6){
                variance_threshold = z_var;
            }

            mean.z = z_mean[0];
            //variance.z = z_var*var_norm;
            mean.intensity = z_mean[0]; variance.intensity = z_var;
            cloud_mean.push_back(mean);
            cloud_var.push_back(variance);
    }
    // publish point cloud
    mean_publisher.publish(cloud_mean);
    variance_publisher.publish(cloud_var);
};

void signal_strength_estimator::process_measurement(tf::StampedTransform &pose_transform, const wifi_mapping::wifi_measurement::ConstPtr &wifi_msg){
    gp_mutex->lock();
    // construct measurement vector, for the moment, we only use translation data
    Eigen::Vector3d pos_x;
    tf::vectorTFToEigen(pose_transform.getOrigin(),pos_x);
    Eigen::VectorXd x(pos_x);

    double ss = wifi_msg->signal_strength; 
    essid = wifi_msg->essid; 
    double prediction_variance;
    Eigen::VectorXd predicted_signal_strength,predictive_error, predictive_variance;

    GP->prediction(x,predicted_signal_strength,prediction_variance);
    GP->predictive_error_and_variance(predictive_error, predictive_variance,0);
    variance_threshold = predictive_variance.maxCoeff();

    ROS_INFO("[%s,%s] Variance Threshold: %f",ap_id.c_str(),wifi_msg->essid.c_str(), variance_threshold);
    publish_clouds();
    ROS_INFO("[%s,%s] Variance Threshold: %f",ap_id.c_str(),wifi_msg->essid.c_str(), variance_threshold);

    ROS_INFO("[%s,%s] Measured Signal Strength: %f, Prediction: %f, Variance: %f, Variance Threshold: %f",ap_id.c_str(),wifi_msg->essid.c_str(),wifi_msg->signal_strength,predicted_signal_strength[0],prediction_variance, variance_threshold);
    bool add_measurement = (prediction_variance > variance_threshold);// || (GP->dataset_size()<5);
    
    if(add_measurement){
        GP->add_sample(x,ss);
        if(GP->dataset_size() > 5){
            //GP->init(GP->kernel->parameters);
            GP->set_opt_random_start(10,10);
            GP->optimize_parameters();
        }
    }
    ROS_INFO("[%s,%s]\t Kernel Params: [%f,%f,%f,%f,%f] likelihood: %f",ap_id.c_str(),
                                                       essid.c_str(),
                                                       GP->kernel->parameters[0],
                                                       GP->kernel->parameters[1],
                                                       GP->kernel->parameters[2],
                                                       GP->kernel->parameters[3],
                                                       GP->kernel->parameters[4],
                                                       GP->log_marginal_likelihood());
    ROS_INFO("[%s,%s]\t Total data points: %d",ap_id.c_str(),wifi_msg->essid.c_str(),GP->dataset_size());
    gp_mutex->unlock();
};

void signal_strength_estimator::optimize(double stopping_criterion, int solver, int restarts, double scale, double offset){
    if(GP->dataset_size() >= 5){
        gp_mutex->lock();
        GP->init(GP->kernel->parameters);
        GP->optimize_parameters_random_restarts(stopping_criterion, solver, restarts, scale, offset);
        gp_mutex->unlock();
    }
};
