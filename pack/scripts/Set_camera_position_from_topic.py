#!/usr/bin/env python
import rospy 
import rospkg 
import signal
import numpy as np
import sys
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R

def signal_handler(sig, frame):
    print('You pressed Ctrl+C! terminating the program...')
    sys.exit(0)

def subscribe_to_pose():
    rospy.init_node('set_pose')
    rospy.Subscriber("/Camera_position", Float64MultiArray, callback)
    rospy.spin()


def callback(msg):
    
    state_msg = ModelState()
    state_msg.model_name = 'camera'
    state_msg.pose.position.x = msg.data[0]
    state_msg.pose.position.y = msg.data[1]
    state_msg.pose.position.z = msg.data[2]
    r = R.from_euler('xyz',msg.data[3:6],degrees=True)
    r = r.as_matrix()
    r = np.matmul(r,np.array([[0,1,0],[0,0,1],[1,0,0]]))
    r = R.from_matrix(r)
    r = r.as_quat()
    state_msg.pose.orientation.x = r[0]
    state_msg.pose.orientation.y = r[1]
    state_msg.pose.orientation.z = r[2]
    state_msg.pose.orientation.w = r[3]

    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException:
        print("Service call failed") 

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    subscribe_to_pose()
