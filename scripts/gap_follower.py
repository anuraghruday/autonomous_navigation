#! /usr/bin/env python3

import rospy
import math
import numpy as np

from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDrive

rospy.init_node("pa2")

car_name =  rospy.get_param("car_name",default="anuraghr_racer")

scan_topic = "/"+ car_name + "/amcl/scan"
cmd_topic = "/"+ car_name + "/command"
imu_topic = "/" + car_name + "/amcl/imu"
odom_topic = "/"+ car_name + "/amcl/odom"

base_link = "/"+car_name + "/base_link"
pose_topic = "/"+car_name+"/amcl/pose"

class Pa2:

    def __init__(self):
  
        self.ranges = None

    def laser_callback(self, data):

        self.angle_min = data.angle_min
        self.angle_max = data.angle_max
        self.angle_increment= data.angle_increment
        self.range_max = data.range_max
        self.ranges = data.ranges

    def cart(self, ranges):

        angle_now = self.angle_min
        ranges_cart = []
        ldistances = []

        for r in self.ranges:

            x = r * math.cos(angle_now)
            y = r * math.sin(angle_now)
            ranges_cart.append((x, y))

            if r == 0.0:
                ldistances.append(10)

            else:
                ldistances.append(r)

            angle_now += self.angle_increment

            # print(ldistances[:5])
        return ldistances, ranges_cart

    def safety_bubble(self, ranges_new):

        self.close_pt_index = np.argmin(ranges_new)

        radius = 0.1

        for i in range(len(ranges_new)):

            if abs(i-self.close_pt_index) < radius:
                ranges_new[i] = 0.0
        return ranges_new
    
    def final_loc(self, gaps, ranges_cart):

        max_distance = 0
        gap_index = -1
   
        for i in range(len(gaps)):
             
            if gaps[i][1] - gaps[i][0] > max_distance:

                max_distance = gaps[i][1] - gaps[i][0]
                gap_index = i
        
        initial_index = gaps[gap_index][0]
        final_index = gaps[gap_index][1]
        output = ranges_cart[initial_index:final_index]
        x, y = output[int(len(output)/2)]

        # print(x,y)
        # print(type(x))

        return x, y

    def far_pts(self, points, ranges_cart):

        initial = None 
        final = None
        gaps = []
        for i in range(len(points)):

            if points[i] > 2:

                if initial == None:
                    initial = i

                else:
                    final = i
            elif initial is not None and final is not None:

                gaps.append([initial, final])
                initial = None
                final = None
        
        x, y = self.final_loc(gaps, ranges_cart)
        return x, y

    def main(self):

        
        rospy.Subscriber(scan_topic, LaserScan, self.laser_callback)
        # self.pub = rospy.Publisher("/ransac_marker", Marker, queue_size=10)

        self.pub = rospy.Publisher(cmd_topic, AckermannDrive, queue_size=10)
        self.rate = rospy.Rate(20)

        self.ack= AckermannDrive()

        while not rospy.is_shutdown():
            
            if self.ranges is not None:

                ldistances, ranges_cart = self.cart(self.ranges)
                ldistances = self.safety_bubble(ldistances)

                x_far, y_far = self.far_pts(ldistances, ranges_cart)
                delta = math.atan(0.648*math.sin(math.atan2(y_far, x_far))/1.25)
                                    
                self.ack.speed = 1.8
                self.ack.steering_angle = -delta
                self.pub.publish(self.ack)
                
                self.rate.sleep()

if __name__ == "__main__":
    try:
        obj = Pa2()
        obj.main()
    except rospy.ROSInterruptException:
        pass