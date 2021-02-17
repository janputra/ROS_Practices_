#!/usr/bin/env python

import rospy

#task 1. import the Pose type from the module turtlesim
from turtlesim.msg import Pose

def poseCallback(pose_message):

   #task 4. display the x, y, and theta received from the message
    x= pose_message.x
    y= pose_message.y
    yaw= pose_message.theta
    print ('pose callback')
    print ('x = ',x) 
    print ('y = ', y) 
    print ('yaw = ',yaw) 

if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_motion_pose', anonymous=True)        

       #task 2. subscribe to the topic of the pose of the Turtlesim
        pose_topic ='/turtle1/pose'
        rospy.Subscriber(pose_topic, Pose, poseCallback)


       #task 3. spin
        rospy.spin()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")