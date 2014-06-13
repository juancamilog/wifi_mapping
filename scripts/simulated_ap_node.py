from ap_simulator import *
import rospy
import tf
from wifi_mapping.msg import wifi_measurement

class simulated_ap:
    def __init__(self):
       
       pass
       
    def publish_wifi_measurement():
        # get location of robot from tf transform
       
        # compute singal strength from location
        
        # populate the wifi_measurement message
        ap_msg = wifi_measurement()

