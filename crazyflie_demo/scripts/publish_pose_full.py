#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import FullState
from catenary_trajectoryA import trajectory



if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    n = rospy.get_param("~Robot", 2)

    r = 50.
    x = 0.5
    y = 0.5
    z = 1.
    t = 0.

    rate = rospy.Rate(r)

    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame

    pub = rospy.Publisher('goal', PoseStamped, queue_size=1)

    while not rospy.is_shutdown():
        [dst_xAd, dst_xBd, Yaw] = trajectory(0)
        # if t < 5:
        #     [dst_xAd, dst_xBd, Yaw] = trajectory(0)
        # elif t >= 5:
        #     [dst_xAd, dst_xBd, Yaw] = trajectory(t-5)

        if n == 0:
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z] = dst_xAd
        else:

            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z] = dst_xBd

        quaternion = tf.transformations.quaternion_from_euler(0, 0, Yaw)
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]

        t = t + 0.01
        #print(t)

        #print(t, dst_xAd)
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()
