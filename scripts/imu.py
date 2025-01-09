#! /usr/bin/env python3

import rospy

from sensor_msgs.msg import Imu

from nav_msgs.msg import Odometry

from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Point, Pose, Twist, Vector3, PoseWithCovariance, TwistWithCovariance


rospy.init_node("imu_test", anonymous=True)

car_name = rospy.get_param("car_name",default="anuraghr_racer")

imu_topic = "/" + car_name + "/amcl/imu"

odom_topic = "/" + car_name + "/amcl/odom"

car_BASE_LINK = "/" + car_name + "/amcl/base_link"

class Imutest:

    def __init__(self) -> None:
        
        self.imu_flag = False

        self.rate = rospy.Rate(10)

        self.prev_time = rospy.Time.now()

        self.last_time = rospy.Time.now()
        self.position = Point(0, 0, 0)
        self.velocity = Vector3(0, 0, 0)

        self.odom_msg = Odometry()
        self.odom_msg.header = Header()

        
        rospy.Subscriber(imu_topic, Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher(odom_topic, Odometry, queue_size=10)
        

    def imu_callback(self,data):

        if data is not None:

            current_time = rospy.Time.now()

            dt = (current_time - self.prev_time).to_sec()

            self.last_time = current_time

            self.velocity.x += data.linear_acceleration.x * dt
            self.velocity.y += data.linear_acceleration.y * dt
            self.velocity.z += data.linear_acceleration.z * dt

            self.position.x += self.velocity.x * dt
            self.position.y += self.velocity.y * dt
            self.position.z += self.velocity.z * dt

            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = "world"
            odom.child_frame_id = car_BASE_LINK

            odom.pose.pose = Pose(self.position, data.orientation)

            odom.twist.twist = Twist(self.velocity, Vector3(0, 0, 0))

            self.odom_pub.publish(odom)

    
    def main(self):

        while not rospy.is_shutdown():
            if self.imu_flag:

                self.rate.sleep()

if __name__ == "__main__":
    obj = Imutest()
    obj.main()