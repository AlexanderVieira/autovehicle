#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
import math
import time
from std_srvs.srv import Empty

x=0
y=0
z=0
yaw=0

def poseCallback(pose_message):
    global x
    global y, z, yaw
    x= pose_message.pose.pose.position.x
    y= pose_message.pose.pose.position.y
    yaw = pose_message.pose.pose.orientation.z

    rospy.loginfo("position x = {}".format(x))
    rospy.loginfo("position y = {}".format(y))
    rospy.loginfo("orientation yaw = {}".format(yaw))


    #print "pose callback"
    #print ('x = {}'.format(pose_message.x)) #new in python 3
    #print ('y = %f' %pose_message.y) #used in python 2
    #print ('yaw = {}'.format(pose_message.theta)) #new in python 3


def move(speed, distance):
        #declare a Twist message to send velocity commands
            velocity_message = Twist()
            #get current location 
            x0=x
            y0=y
            #z0=z;
            #yaw0=yaw;
            velocity_message.linear.x =speed
            distance_moved = 0.0
            loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
            cmd_vel_topic='/cmd_vel'
            velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

            while True :
                    rospy.loginfo("Autovehicle moves forwards")
                    velocity_publisher.publish(velocity_message)

                    loop_rate.sleep()
                    
                    #rospy.Duration(1.0)
                    
                    distance_moved = distance_moved+abs(0.5 * math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))

                    rospy.loginfo("distance_moved = {}".format(distance_moved))
                    rospy.loginfo("distance = {}".format(distance))

                    rospy.loginfo(distance_moved)
                    
                    if  not (distance_moved<distance):
                        rospy.loginfo("reached")
                        break
            
            #finally, stop the robot when the distance is moved
            velocity_message.linear.x =0
            velocity_publisher.publish(velocity_message)
    


if __name__ == '__main__':
    try:
        
        rospy.init_node('autovehicle_motion_pose', anonymous=True)

        #declare velocity publisher
        cmd_vel_topic='/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = "/odom"
        pose_subscriber = rospy.Subscriber(position_topic, Odometry, poseCallback) 
        time.sleep(2)        
        rospy.loginfo("move")
        move (-1.0, 10.0)
        time.sleep(2)        
        print('start reset: ') 
        rospy.wait_for_service('reset')
        reset_turtle = rospy.ServiceProxy('reset', Empty)
        reset_turtle()
        print('end reset: ') 
        rospy.spin()
       
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")