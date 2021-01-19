#!/usr/bin/env python

import rospy
import tf
import numpy as np
from geometry_msgs.msg import PoseStamped
from crazyflie_driver.msg import FullState
from catenary_trajectoryC import trajectoryC # you can choose wich experiment you want to use
from trajectory import min_snap_trajectory


# This code is used for computing our three experiments,
# depending of the experiment you need to import one diferent file
# choosing trajectoryX where X = {A,B,C}, in each file is computed a diferent
# type of trajectory


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

    wayptset =  np.array([[-1.6,-0.1,0.6],
                          [0.0,-0.2, 0.6],
                          [0.6, 0.17, 0.509],
                          [0.8,0.7,1.]])
    speed = 15
    traj_vars = min_snap_trajectory(0, speed, None, waypts=wayptset)
    # msg = FullState()
    # msg.header.seq = 0
    # msg.header.stamp = rospy.Time.now()
    # msg.header.frame_id = "/world"
    #
    #
    # pub = rospy.Publisher("cmd_full_state", FullState, queue_size=1)

    tend = 8

    while not rospy.is_shutdown():

        # [pos, vel, acc, yaw, yawdot] = min_snap_trajectory(t, speed, traj_vars)
        # [dst_xAd, dst_xBd, Yaw] = trajectoryC(t,pos,yaw)


        if t < tend:
            [dst_xAd, dst_xBd, Yaw] = trajectoryC(0,np.array([-1.6,-0.1,0.6], dtype='f'),0)
        elif t >= tend:
            if t-tend<= speed:
                [pos, vel, acc, yaw, yawdot] = min_snap_trajectory(t-tend, speed, traj_vars)
                print(pos)

            [dst_xAd, dst_xBd, Yaw] = trajectoryC(t-tend, pos, yaw)

        if n == 0:
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z] = dst_xAd
            # [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z] = [0,0,0]
            # [msg.acc.x, msg.acc.y, msg.acc.z] = [0,0,0]
        else:

            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z] = dst_xBd
            # [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z] = [0,0,0]
            # [msg.acc.x, msg.acc.y, msg.acc.z] = [0,0,0]



        quaternion = tf.transformations.quaternion_from_euler(0, 0, Yaw)
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]

        t = t + 0.01


        #print(t, dst_xAd)
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()
