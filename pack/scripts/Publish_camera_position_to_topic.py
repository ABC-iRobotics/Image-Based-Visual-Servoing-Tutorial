#!/usr/bin/env python
from std_msgs.msg import Float64MultiArray
import signal
import sys
import rospy 
import rospkg 
import numpy as np

    
def publish_camera_pose(Camera_pose):
    pub = rospy.Publisher('/Camera_position', Float64MultiArray, queue_size=10)
    rospy.init_node('set_pose_once', anonymous=True)
    rate = rospy.Rate(100)
    my_msg = Float64MultiArray()
    my_msg.data = Camera_pose.flatten()
    my_msg.layout.data_offset =  0
    pub.publish(my_msg)    
    
if __name__ == '__main__':
    values = np.array([0.01,-0.015,0.5,179,2,30])
    publish_camera_pose(values)
