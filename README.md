# Image-Based-Visual-Servoing-Tutorial
This repository is a resource for anyone interested in implementing the basics of image-based visual servoing methods . With the help of the included scripts and files, you'll be able to quickly create an implementation.

## Implementation
### Prerequirest
The Gazebo simulation environment and the ROS environment and the integration of these two are required to use this method. (Used ROS version: Noetic, Gazebo version:11.12.0)

You can use the following link to install these:https://classic.gazebosim.org/tutorials?tut=install_ubuntu

After the installation you can build the 'pack' package.

### Using the package

Starting roscore to use ROS communication:

    roscore

Launch the Gazebo world file:

    roslaunch pack launcher.launch
    
Setup a topic to adjust camera position:

    rosrun pack Set_camera_position_from_topic.py
    
Set the camera position to the hard coded coordinates, this position will be the desired position:

    rosrun pack Publish_camera_position_to_topic.py
    
Create desired input values (image,Z value):

    rosrun pack Feature_image.py
    
Change hard coded camera position in Publish_camera_position_to_topic.py to the starting position, and then rerun that. 

    rosrun pack Publish_camera_position_to_topic.py
    
Start image based visual servoing method:

    rosrun pack IBVS.py
    
## Example

Running the following commands in separated command windows:

    roscore
    roslaunch pack launcher.launch
    rosrun pack Set_camera_position_from_topic.py
    
Define the coordinates in Publish_camera_position_to_topic.py

    if __name__ == '__main__':
    values = np.array([0,0,0.3,180,0,0]) # desired camera position for now
    publish_camera_pose(values)

Create desired input values (image,Z value):

    rosrun pack Feature_image.py
    
Define the coordinates in Publish_camera_position_to_topic.py

    if __name__ == '__main__':
    values = np.array([0.015,-0.01,0.5,179,2,30]) #starting camera position
    publish_camera_pose(values)
    
Start image based visual servoing method:

    rosrun pack IBVS.py
    
The movement of the camera object can be seen on the Gazebo simulation window (in the eaxmple from position 0 to position 6, along the white arrow), and the error vector can be seen on the commander:

![Camera_movement_v1](https://github.com/ArminKaroly/Image-Based-Visual-Servoing-Tutorial/assets/41468450/994e9e5e-3c01-42d6-903c-7c8749459f9e)

The camera position during the method: 

![](https://i.imgur.com/eHNBvPn.png)


