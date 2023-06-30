#! /usr/bin/python
import rospy
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates
from scipy.spatial.transform import Rotation as R
import numpy as np
from cv_bridge import CvBridge
import cv2
import sys
import os


bridge = CvBridge()
class Datas:
  def image_callback(self,msg):
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imwrite('Feature_image.jpg',cv2_img)
        
  def position_callback(self,msg):
      
      val = 0
      for i in range(0,len(msg.name)):

          if(msg.name[i]=='camera'):
              val = i
      cam_coords = msg.pose[val]   
      r = R.from_quat(np.array([cam_coords.orientation.x,cam_coords.orientation.y,cam_coords.orientation.z,cam_coords.orientation.w]))
      r = r.as_matrix()
      r = np.matmul(r,np.array([[0,0,1],[1,0,0],[0,1,0]]))
      r = R.from_matrix(r)
      r = r.as_euler('xyz',degrees = True)
      camera_coords = np.array([cam_coords.position.x,cam_coords.position.y,cam_coords.position.z,r[0],r[1],r[2]])      
      file1 = open('Feature_image.txt', 'w')
      file1.writelines(str(camera_coords[2]))
      
if __name__ == '__main__':
    os.chdir(sys.path[0])
    rospy.init_node('IBVS')
    ibvs = Datas()
    rospy.Subscriber('/gazebo/model_states', ModelStates, ibvs.position_callback)
    rospy.Subscriber('/camera1/image_raw', Image, ibvs.image_callback)
    rospy.sleep(0.1)
    quit()
    
    
    
