#!/usr/bin/env python
import rospy
import math
import tf2_ros
import geometry_msgs.msg
import pcl_ros
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from roslib import message
import pdb

def callback_kinect(data) :
    # pick a height
    print(data.header)
    # height =  int (data.height / 2)
    # pick x coords near front and center
    # middle_x = int (data.width / 2)
    # examine point
    # middle = read_depth (middle_x, height, data)

    # do stuff with middle

if __name__ == '__main__':
    rospy.init_node('depth_uav_to_world')
    rate = rospy.Rate(10)
    tfBuffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tfBuffer)

    rospy.Subscriber("camera/depth_registered/points", PointCloud2, callback_kinect)

    while not rospy.is_shutdown():
        print('tf received')
        try:
            trans = tfBuffer.lookup_transform('camera_rgb_optical_frame','world', rospy.Time())
            data_world = trans.transformPointCloud('world', data)
            # print(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
        rate.sleep()
