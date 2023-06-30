#!/usr/bin/env python
import os
import sys
import math
import time
import scipy
import rospy
import cv2 as cv
import numpy as np
from os import system, name
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
#from csv import writer
#import matplotlib.pyplot as plt

bridge = CvBridge()
##################################
# Used class
##################################
class Datas:
    def __init__(self,timestep,P,epsilon):
        self.image = None
        self.camera_coords = None
        self.camera_parameters = None
        self.Feature_image = cv.imread("Feature_image.jpg")
        file1 = open('Feature_image.txt', 'r')
        line = file1.readlines()
        self.Feature_Z = float(line[0])
        self.timestep = timestep
        self.P = P
        self.epsilon = epsilon      
            
    def IBVS(self,msg):

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
            if ((self.camera_coords != np.array([cam_coords.position.x,cam_coords.position.y,cam_coords.position.z,r[0],r[1],r[2]])).all()):
              self.camera_coords = np.array([cam_coords.position.x,cam_coords.position.y,cam_coords.position.z,r[0],r[1],r[2]])

              #with open('output.csv','a') as csvfile:
              #  np.savetxt(csvfile, np.reshape(self.camera_coords,(1,6)),delimiter=',',fmt='%f')
              projected_desired = self.Corner_det(self.Feature_image)
              projected_current = self.Corner_det(self.image)
            
              p_mm_desired = self.Pixel_to_mm(self.camera_parameters,projected_desired)
              p_mm_current = self.Pixel_to_mm(self.camera_parameters,projected_current)       

              sc = p_mm_desired.flatten()
              s = p_mm_current.flatten()
              error = s-sc
              
              if np.sum(abs(error))<=self.epsilon:
                print("Hiba elhanyagolhato")
                os._exit(0)
              else:
                Interaction_matrix_desired = np.zeros((projected_desired.shape[0]*2,6));
                for i in range(projected_desired.shape[0]):
                    Interaction_matrix_desired[2*i:2*i+2,:] = self.Create_Lx(self.Feature_Z,p_mm_desired[i,:])
              
                Interaction_matrix_current = np.zeros((projected_current.shape[0]*2,6));
                for i in range(projected_current.shape[0]):
                    Interaction_matrix_current[2*i:2*i+2,:] = self.Create_Lx(self.camera_coords[2],p_mm_current[i,:])

              
                Interaction_matrix = (Interaction_matrix_desired +Interaction_matrix_current)/2
                Interaction_matrix_inv = np.linalg.pinv(Interaction_matrix)
                vc = -self.P*Interaction_matrix_inv@np.matrix.transpose(error)
                
                rotation_matrix = R.from_euler('xyz',self.camera_coords[3:6],degrees = True).as_matrix()
                rotation_pos = np.array([[-1,0,0],[0,-1,0],[0,0,1]])
                rotation_ori = np.array([[-1,0,0],[0,1,0],[0,0,-1]])
                
                Linear_vec = np.reshape(rotation_pos@rotation_matrix@np.reshape(vc[0:3],(3,1)),(1,3))
                Rotational_vec = np.reshape(rotation_ori@np.reshape(vc[3:6],(3,1)),(1,3))  
                
                t = (self.camera_coords[0:3] + np.reshape(Linear_vec,(1,3))*self.timestep)    
                state_msg = ModelState()
                state_msg.model_name = 'camera'
                state_msg.pose.position.x = t[0,0]
                state_msg.pose.position.y = t[0,1]
                state_msg.pose.position.z = t[0,2]
                r = self.camera_coords[3:6] + np.reshape(Rotational_vec,(1,3))*self.timestep*180
                r = R.from_euler('xyz',r,degrees=True).as_matrix()
                r = np.matmul(r,np.array([[0,1,0],[0,0,1],[1,0,0]]))
                r = R.from_matrix(r).as_euler('xyz',degrees=True)
                q = R.from_euler('xyz',r,degrees = True).as_quat()
                state_msg.pose.orientation.x = q[0,0]
                state_msg.pose.orientation.y = q[0,1]
                state_msg.pose.orientation.z = q[0,2]
                state_msg.pose.orientation.w = q[0,3]
                rospy.wait_for_service('/gazebo/set_model_state')
                try:
                    set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
                    resp = set_state( state_msg )
                    print(error)
                except rospy.ServiceException:
                    print("Camera position cannot be set!")
                          
    def image_callback(self, msg):
        self.image = bridge.imgmsg_to_cv2(msg, "bgr8")
        time.sleep(0.05)
        
    def camera_parameters_callback(self, msg):
        self.camera_parameters = msg
        time.sleep(0.05)    
                                
    def Pixel_to_mm(self,camera_parameters,D2_points):
        output = D2_points
        Camera_matrix = np.reshape(camera_parameters.K,(3,3))
        for i in range(D2_points.shape[0]):
            Point_mm = np.matrix.transpose(np.linalg.inv(Camera_matrix)@np.reshape(np.append(D2_points[i,:],1),(3,1)))
            output[i,:] = Point_mm[0,0:2]
        return output

    def Create_Lx(self,Z,D2_point):
        Lx = np.array([[-1/Z , 0 , D2_point[0]/Z , D2_point[0]*D2_point[1] , -(1+D2_point[0]**2) , D2_point[1]],\
                         [0 , -1/Z , D2_point[1]/Z , 1+D2_point[1]**2 , -D2_point[0]*D2_point[1] , -D2_point[0]]])
        return Lx
 
 
    def Corner_det(self,img):
        lowerk = np.array([200, 0, 0], dtype = "uint8") 
        upperk= np.array([255, 100, 100], dtype = "uint8")
        lowerz = np.array([0, 150, 0], dtype = "uint8") 
        upperz= np.array([100, 255, 100], dtype = "uint8")
        lowerp = np.array([0, 0, 200], dtype = "uint8") 
        upperp= np.array([100, 100, 255], dtype = "uint8")
        lowerf = np.array([0,0,0], dtype = "uint8")
        upperf = np.array([10,10,10], dtype = "uint8")

        thk = cv.inRange(img,lowerk,upperk)
        thz = cv.inRange(img,lowerz,upperz)
        thp = cv.inRange(img,lowerp,upperp)
        thf = cv.inRange(img,lowerf,upperf)

        kek_sp = scipy.ndimage.center_of_mass(thk)
        zold_sp = scipy.ndimage.center_of_mass(thz)
        piros_sp = scipy.ndimage.center_of_mass(thp)
        fekete_sp = scipy.ndimage.center_of_mass(thf)
             
        corners = np.array([[piros_sp[1],piros_sp[0]],[zold_sp[1],zold_sp[0]],[kek_sp[1],kek_sp[0]],[fekete_sp[1],fekete_sp[0]]])
        return corners      
                
##################################
##################################                     
if __name__ == '__main__':
    os.chdir(sys.path[0])
    rospy.init_node('IBVS')
    ibvs = Datas(1/60,3,0.01)
    rospy.Subscriber('/camera1/image_raw', Image, ibvs.image_callback)
    rospy.Subscriber('/camera1/camera_info', CameraInfo, ibvs.camera_parameters_callback)
    rospy.Subscriber('/gazebo/model_states', ModelStates, ibvs.IBVS)
    rospy.spin()
