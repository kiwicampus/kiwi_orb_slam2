#!/usr/bin/env python
import sys
import os
import time

import rospy 

from std_msgs.msg import Bool
from std_msgs.msg import Header
from odometry.msg import EncoderOdom

from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler


class SubscriberClass(object):

    def __init__(self, suscribe_topic, suscribe_type, queue=10):
        rospy.Subscriber(suscribe_topic, suscribe_type, self._suscribe_cb, queue_size = queue)
        self._data = None

        # Publish something
        self.pub = rospy.Publisher("/kiwibot/pose_stamped", PoseStamped, queue_size=10) #Status is our own type defined in ./msg/Status.msg

        self.pose_msg = PoseStamped()
        self.raw_pose = Pose()
        self.header = Header()
        self.header.frame_id = "orb_slam2/world"

        self.x = 0
        self.y = 0
        self.timestamp = 0

        self.yaw_offset = 0

    def _suscribe_cb(self, data):
        self._data = data

        x = data.x_dot
        y = data.y_dot

        if self.timestamp != 0:
            current_time = rospy.Time.from_sec(time.time()).to_sec()
            delta_t = current_time - self.timestamp
            self.timestamp = current_time
            
            self.x += delta_t*x
            self.y += delta_t*y
        else:
            self.timestamp = rospy.Time.from_sec(time.time()).to_sec()
            self.yaw_offset = data.theta

        r = data.roll 
        p = data.pitch 
        y = data.theta - self.yaw_offset

        self.header.stamp = rospy.Time.now()

        q = quaternion_from_euler(r,0,y)

        # self.raw_pose.position = Point(self.x,self.y,0.0)
        self.raw_pose.position = Point(-self.y,0.0,self.x)
        self.raw_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        self.pose_msg.header = self.header
        self.pose_msg.pose = self.raw_pose
        self.pub.publish(self.pose_msg)



    @property
    def data(self):
        return self._data


def main():
    # In ROS, nodes are uniquely named. If two nodes with the same name are launched, the previous one is kicked off.
    rospy.init_node('odom_node', anonymous=True)
    rate = 2 # rate in Hertz

    # Init Objects
    odom = SubscriberClass("/kiwibot/odometry/estimation", EncoderOdom)

    # Publish something
    pub = rospy.Publisher("/kiwibot/pose_stamped", PoseStamped, queue_size=10) #Status is our own type defined in ./msg/Status.msg


    rate = rospy.Rate(rate)

    pose_msg = PoseStamped()
    raw_pose = Pose()
    header = Header()
    header.frame_id = "orb_slam2/world"


    while not rospy.is_shutdown():

        # DO STUFF

        rate.sleep()

        # r = odom.data.roll 
        # p = odom.data.pitch 
        # y = odom.data.theta

        # header.stamp = rospy.Time.now()

        # q = quaternion_from_euler(r,0,y)

        # raw_pose.position = Point(0.0,0.0,0.0)
        # raw_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        # pose_msg.header = header
        # pose_msg.pose = raw_pose
        # pub.publish(pose_msg)



# tf::Quaternion q(imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w);
# tf::Matrix3x3 m(q);
# m.getRPY(imu_pitch, imu_roll, imu_yaw);


if __name__ == '__main__':
    main()
    # try:
    #     main()
    # except rospy.ROSInterruptException:
    #     pass
    # except:
    #     print(sys.exc_info())
