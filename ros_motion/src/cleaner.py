#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time
from std_srvs.srv import Empty


def poseCallback(pose_message):
    global x
    global y, z, yaw
    x= pose_message.x
    y= pose_message.y
    yaw = pose_message.theta

def euclidean_dist(x0,x1,y0,y1):
    return abs(math.sqrt(((x1-x0)**2)+((y1-y0)**2)))

def angle_two_points(x0,x1,y0,y1):
    return math.atan2(y1-y0,x1-x0)

def move(publisher, speed, distance, is_forward):
    
    vel_msgs= Twist()

    if (is_forward):
        vel_msgs.linear.x= abs(speed)
        info = "Turtlesim moves forward"
    else:
        vel_msgs.linear.x=-abs(speed)
        info = "Turtlesim moves backward"

    #get current location
    x0=x
    y0=y

    distance_moved=0
    loop_rate= rospy.Rate(100)

    rospy.loginfo(info)

    while True :

        

        publisher.publish(vel_msgs)
        loop_rate.sleep()

        distance_moved = euclidean_dist(x0,x,y0,y)
        print('distance_moved : ', distance_moved)

        if not (distance_moved<distance):
            rospy.loginfo("reached")
            break
    
    vel_msgs.linear.x=0
    publisher.publish(vel_msgs)

def rotate(publisher, angular_speed_degree, relative_angle_degree, clockwise):
    
    vel_msgs= Twist()

    angular_speed= math.radians(angular_speed_degree)

    if (clockwise):
        vel_msgs.angular.z = -abs(angular_speed)
        info = "Turtlesim rotates clockwise"
    else:
        vel_msgs.angular.z = abs(angular_speed)
        info = "Turtlesim rotates counter clockwise"

    t0=rospy.Time.now().to_sec()  
    loop_rate= rospy.Rate(100)

    rospy.loginfo(info)

    while True :

        
        publisher.publish(vel_msgs)
       
        t1=rospy.Time.now().to_sec()

        current_angle= (t1-t0)*angular_speed_degree
        print('Relative angle : ', current_angle)
        loop_rate.sleep()

        if not (current_angle<relative_angle_degree):
            rospy.loginfo("reached")
            break
    
    vel_msgs.angular.z=0
    publisher.publish(vel_msgs)

def go_to_goal(publisher, x_goal, y_goal):
    
    vel_msgs = Twist()

    kp_lin=1
    kp_ang=4

    loop_rate= rospy.Rate(100)
    while True:

        dist= euclidean_dist(x,x_goal,y,y_goal)
        linear_speed= dist*kp_lin

        desired_angle=angle_two_points(x,x_goal,y,y_goal)
        angular_speed = (desired_angle-yaw)*kp_ang

        vel_msgs.linear.x=linear_speed
        vel_msgs.angular.z= angular_speed

        publisher.publish(vel_msgs)
        loop_rate.sleep()
        print('x=',x ,'y=',y, 'distance to goal=',dist, 'theta', yaw)

        if (dist<0.01):
            break
    
def setOrientation(publisher, speed_degreee,desired_orientation):
    
    relative_angle_rad= math.radians(desired_orientation)-yaw
    if (relative_angle_rad>math.radians(180)):
        relative_angle_rad=relative_angle_rad-math.radians(360)
    
    if (relative_angle_rad>0):
        clockwise=0
    else:
        clockwise=1
    
    print('relative angle=', math.degrees(relative_angle_rad),'desired orientation=', desired_orientation)
    rotate(publisher,speed_degreee,math.degrees(abs(relative_angle_rad)),clockwise)
  
def spiral(publisher, rk, wk):
    vel_msgs= Twist()

    loop_rate =rospy.Rate(1)

    while((x<10) and (y<10)):
        rk=rk+0.5
        vel_msgs.linear.x=rk
        vel_msgs.angular.z=wk
        publisher.publish(vel_msgs)
        loop_rate.sleep()
        print('linear speed=',rk, 'theta', math.degrees(yaw))
    
    vel_msgs.linear.x=0
    vel_msgs.angular.z=0
    publisher.publish(vel_msgs)
    
def grid_motion(publisher):

    go_to_goal(publisher,1,1)

    setOrientation(publisher,30,90)

    for i in range(5):
        move(publisher,2.0,9.0,True)
        rotate(publisher,20,90,True)
        move(publisher,2.0,1.0,True)
        rotate(publisher,20,90,True)
        move(publisher,2.0,9.0,True)
        rotate(publisher,20,90,False)
        move(publisher,2.0,1.0,True)
        rotate(publisher,20,90,False) 
    
def turtle_reset():
    print('start reset')
    rospy.wait_for_service('reset')
    reset_turtle = rospy.ServiceProxy('reset', Empty)
    reset_turtle()
    print('end reset')

if __name__=='__main__':

    try:
        rospy.init_node('clean_node',anonymous=True)

        vel_topic= '/turtle1/cmd_vel' 
        velocity_publisher = rospy.Publisher(vel_topic,Twist,queue_size=10)

        pose_topic='/turtle1/pose'
        pose_subscriber=rospy.Subscriber(pose_topic,Pose,poseCallback)

        time.sleep(2)

        while True:
            cmd=input("Input Command (grid, spiral, reset, exit) : \n")

            if (cmd =="grid"):
                grid_motion(velocity_publisher)
            elif(cmd=="spiral"):
                spiral(velocity_publisher,0,2)
            elif(cmd=="reset"):
                turtle_reset()
            elif(cmd=="exit"):
                break
        
        #move(velocity_publisher, 1, 4,False)
        #rotate(velocity_publisher,30,90,True)
        #go_to_goal(velocity_publisher,1,1)
        #go_to_goal(velocity_publisher,5,5)
        #go_to_goal(velocity_publisher,1,1)
        #go_to_goal(velocity_publisher,5,5)
        #setOrientation(velocity_publisher,5,0)
        #spiral(velocity_publisher,0,2)
        #turtle_reset()


    except rospy.ROSInterruptException:
         rospy.loginfo("node terminated.")
