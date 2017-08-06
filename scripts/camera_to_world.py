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

temp = PointCloud2()
def callback_kinect(data) :
    # pick a height
    print(data.header)
    temp = data
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
        # print('tf received')
        try:
            trans = tfBuffer.lookup_transform('camera_rgb_optical_frame','world', rospy.Time())
            # tf_listener.waitForTransform('camera_rgb_optical_frame','world',rospy.Time(),rospy.Duration(1.0))
            tf_listener.transformPointCloud('world', temp)
            # print(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue
        # tf_listener.transformPointCloud('world', data)
        rate.sleep()
