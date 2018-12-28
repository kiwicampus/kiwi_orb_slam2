#!/usr/bin/env python
import sys
import os
import time

import rospy 

from math import sin, cos

from std_msgs.msg import Bool, Float32
from std_msgs.msg import Header
from odometry.msg import EncoderOdom

from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from tf.msg import tfMessage
from tf.transformations import quaternion_from_euler

import csv


def create_csv(name):
    csv_file = os.path.join(name)

    with open(csv_file, 'w') as fd:
        writer = csv.writer(fd)
        row = ['x', 'y', 'timestamp']
        writer.writerow(row)
    return name

def write_row(file_name, row):
    with open(file_name, 'a') as fd:
        writer = csv.writer(fd)
        writer.writerow(row)

class SubscriberClass(object):

    def __init__(self, suscribe_topic, suscribe_type, queue=10):
        rospy.Subscriber(suscribe_topic, suscribe_type, self._suscribe_cb, queue_size = queue)

        rospy.Subscriber("/tf", tfMessage, self._tf_cb, queue_size = queue)
        self._data = None

        # Publish something
        self.pub = rospy.Publisher("/kiwibot/pose_stamped", PoseStamped, queue_size=10) #Status is our own type defined in ./msg/Status.msg
        
        self.pubx = rospy.Publisher("/kiwibot/odom/x", Float32, queue_size=10)
        self.puby = rospy.Publisher("/kiwibot/odom/y", Float32, queue_size=10)

        self.pub_orbx = rospy.Publisher("/orb/odom/x", Float32, queue_size=10)
        self.pub_orby = rospy.Publisher("/orb/odom/y", Float32, queue_size=10)
        self.pub_orbz = rospy.Publisher("/orb/odom/z", Float32, queue_size=10)

        self.pose_msg = PoseStamped()
        self.raw_pose = Pose()
        self.header = Header()
        self.header.frame_id = "orb_slam2/world"

        self.x = 0
        self.y = 0
        self.timestamp = 0

        self.yaw_offset = 0

        # self.imu_file = create_csv("imu_odom.csv")
        self.orb_file = create_csv("orb_odom.csv")


    def _tf_cb(self, data):
        child_frame = data.transforms[0].child_frame_id
        if child_frame == '/orb_slam2/camera':
            tras = data.transforms[0].transform.translation

            self.pub_orbx.publish(tras.x)
            self.pub_orby.publish(tras.y)
            self.pub_orbz.publish(tras.z)
            
            timestamp = rospy.Time.from_sec(time.time()).to_sec()
            write_row(self.orb_file, [tras.z,tras.x, timestamp])

    def _suscribe_cb(self, data):
        self._data = data

        x_dot = data.x_dot
        y_dot = data.y_dot


        if self.timestamp != 0:
            current_time = rospy.Time.from_sec(time.time()).to_sec()
            delta_t = current_time - self.timestamp
            self.timestamp = current_time
            
            theta = data.theta - self.yaw_offset
            x_dot = data.vx * cos(theta) 
            y_dot = data.vx * sin(theta)

            self.x += delta_t*x_dot
            self.y += delta_t*y_dot
        else:
            self.timestamp = rospy.Time.from_sec(time.time()).to_sec()
            self.yaw_offset = data.theta

        r = data.roll 
        p = data.pitch 
        y = data.theta - self.yaw_offset

        self.header.stamp = rospy.Time.now()

        q = quaternion_from_euler(r,-y-3.1416/2,0)
        #Z -> X
        #X -> -Y
        # self.raw_pose.position = Point(self.x,self.y,0.0)
        self.raw_pose.position = Point(-self.y,0.0,self.x)
        self.raw_pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        self.pose_msg.header = self.header
        self.pose_msg.pose = self.raw_pose
        self.pub.publish(self.pose_msg)

        self.pubx.publish(self.x)
        self.puby.publish(-self.y)

        # write_row(self.imu_file, [self.x,-self.y, self.timestamp])



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
