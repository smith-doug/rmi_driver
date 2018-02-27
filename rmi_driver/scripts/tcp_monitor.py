#!/usr/bin/env python
import rospy

import math
import tf2_ros
import tf2_py
import tf
import numpy

import robot_movement_interface as rmi

import geometry_msgs.msg


if __name__ == '__main__':
    rospy.init_node('tf2_turtle_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    trans = geometry_msgs.msg.TransformStamped()
    trans_tool_world = geometry_msgs.msg.TransformStamped()

    rate = rospy.Rate(10.0)

    max_dist = 0
    max_allowed_dist = 0.002

    msg_sent = False

    while not rospy.is_shutdown():
        try:
            trans_tool_world = tfBuffer.lookup_transform('world', 'tool_frame_pose', rospy.Time())

#             trans = tfBuffer.lookup_transform('tcp_frame', 'tool_frame_pose', rospy.Time(
#                 secs=trans_tool_world.header.stamp.secs, nsecs=trans_tool_world.header.stamp.nsecs))
            trans = tfBuffer.lookup_transform('tcp_frame', 'tool_frame_pose', rospy.Time.now() - rospy.Duration(0.1))

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as ex:
            print ex
            rate.sleep()
            continue

        pos = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
        dist = numpy.linalg.norm(pos)

        if dist > max_dist:
            max_dist = dist

        if dist > max_allowed_dist:
            print dist

        print dist

        rate.sleep()


#         msg = geometry_msgs.msg.Twist()
#
#         msg.angular.z = 4 * math.atan2(trans.transform.translation.y, trans.transform.translation.x)
#         msg.linear.x = 0.5 * math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
#
#         print msg
