#!/usr/bin/env python2
from radio_model import *
import rospy
import tf
from wifi_mapping.msg import wifi_measurement
import json

class simulated_ap:
    def __init__(self, base_frame_id, fixed_frame_id):
       self.publisher = rospy.Publisher("/wifi_measurement",wifi_measurement, queue_size=1)

       self.access_point_model = radio_model()
       # get radio propagation model type
       model_type = rospy.get_param("~radio_model","log_distance")
       if model_type == "log_distance":
            params = json.loads(rospy.get_param("~model_params", '{"location": [5,5,0], "transmit_power": 20, "path_loss_exponent": 2}'))
            self.access_point_model.set_function(path_loss_w, params)
       elif model_type == "trained_gp":
            pass
       pass

       self.tf_listener = tf.TransformListener()

       self.pose_frame_id = base_frame_id
       self.world_frame_id = fixed_frame_id
       
       try:
           self.tf_listener.waitForTransform(self.world_frame_id, self.pose_frame_id, rospy.Time.now(), rospy.Duration(1.0));
       except tf.Exception:
           pass
       
    def publish_wifi_measurement(self):
        # get location of robot from tf transform
        now = rospy.Time.now()
        try:
            self.tf_listener.waitForTransform(self.world_frame_id, self.pose_frame_id, now, rospy.Duration(1.0));
            (trans,rot) = self.tf_listener.lookupTransform(self.world_frame_id, self.pose_frame_id, now)
       
            # compute singal strength from location
            rss = self.access_point_model.get_rss(trans)
            rospy.loginfo(trans)
            rospy.loginfo(self.access_point_model.params['location'])
        except tf.Exception:
            rss = -127

        # populate the wifi_measurement message
        ap_msg = wifi_measurement()
        ap_msg.id = rospy.get_name()[1:]
        ap_msg.signal_strength = rss
        ap_msg.noise = 0
        ap_msg.channel = 0

        self.publisher.publish(ap_msg)

if __name__=='__main__':
    #rospy.init_node('access_point', anonymous=True)
    rospy.init_node('access_point')
    rate_hz = rospy.get_param("rate_hz", 0.5)
    r = rospy.Rate(rate_hz)
    base_frame_id = rospy.get_param("~base_frame_id", 'base_footprint')
    fixed_frame_id = rospy.get_param("~fixed_frame_id", 'world')
    ap = simulated_ap(base_frame_id, fixed_frame_id)

    while not rospy.is_shutdown():
        ap.publish_wifi_measurement()
        r.sleep()
