#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import Point,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

rospy.init_node("pas51b",anonymous=True)

car_name =  rospy.get_param("car_name",default="anuraghr_racer")

print(car_name)
scan_topic = "/"+ car_name + "/amcl/scan"
cmd_topic = "/"+ car_name + "/command"
imu_topic = "/" + car_name + "/amcl/imu"
odom_topic = "/"+ car_name + "/amcl/odom"

base_link = "/"+car_name + "/base_link"
pose_topic = "/"+car_name+"/amcl/pose"


class Pa51b:
    def __init__(self) -> None:

        rospy.init_node("pas51b",anonymous=True)

        self.rate = rospy.Rate(10)

        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'world'
        self.odom_msg.child_frame_id = base_link

        rospy.Subscriber(pose_topic,PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber(imu_topic, Imu, self.imu_callback)
        self.odom_publisher = rospy.Publisher(odom_topic, Odometry, queue_size=10)

    def pose_callback(self,pose_msg):

        self.odom_msg.header.stamp = rospy.Time.now()
        # self.odom_msg.header = pose_msg.header
        self.odom_msg.pose = pose_msg.pose


    def imu_callback(self, imu_msg):
        
        self.odom_msg.twist.twist.linear.x = imu_msg.linear_acceleration.x
        self.odom_msg.twist.twist.angular.z = imu_msg.angular_velocity.z

        self.odom_publisher.publish(self.odom_msg)

    def main(self):

        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == '__main__':
    try:
        obj = Pa51b()
        obj.main()
    except rospy.ROSInterruptException:
        pass