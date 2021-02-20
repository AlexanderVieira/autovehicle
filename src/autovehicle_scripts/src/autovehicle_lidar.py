#!/usr/bin/env python3
#################################################################################
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Author: Alexander Silva #

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

LINEAR_VEL = 1.0
STOP_DISTANCE = 0.2
LIDAR_ERROR = 0.03
SAFE_STOP_DISTANCE = STOP_DISTANCE + LIDAR_ERROR
NUM_READINGS = 10
distance = None
scan_filter =[]

class Obstacle():

    

    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.lidar_sub = rospy.Subscriber('/ir_range/scan', Range, self.get_scan, queue_size = 1)
        self.lidar()
        self.obstacle()

    def get_scan(self, sensor):
        
        global scan_filter, distance
        #twist = Twist()
        distance = sensor.range
        rospy.loginfo('Distance of the obstacle : %f', distance)
        
        scan_filter_len = len(scan_filter)
        rospy.loginfo('scan_filter_len : %d', scan_filter_len)

        while(scan_filter_len < NUM_READINGS):            
            scan_filter.append(distance)
        
        # if distance < 0.20:
        #     linear_vel = 0
        # else:
        #     linear_vel = 1.0

        # twist.linear.x = linear_vel
        # self.cmd_pub.publish(twist) 

    def lidar(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def obstacle(self):
        global scan_filter
        twist = Twist()
        autovehicle_moving = True

        while not rospy.is_shutdown():
            #lidar_distances = self.get_scan()
            min_distance = min(scan_filter)
            rospy.loginfo('min_distance: %f', min_distance) 

            if min_distance < SAFE_STOP_DISTANCE:
                if autovehicle_moving:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    autovehicle_moving = False
                    rospy.loginfo('Stop!')
            else:
                twist.linear.x = LINEAR_VEL
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                autovehicle_moving = True
                rospy.loginfo('Distance of the obstacle : %f', min_distance)            

def main():
    rospy.init_node('autovehicle_lidar')
    try:
        obstacle = Obstacle()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
