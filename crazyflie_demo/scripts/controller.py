#!/usr/bin/env python
#
# import rospy
# from sensor_msgs.msg import Joy
# from crazyflie_driver.srv import UpdateParams
# from std_srvs.srv import Empty
#
# class Controller():
#     def __init__(self, use_controller, joy_topic):
#         rospy.wait_for_service('update_params')
#         rospy.loginfo("found update_params service")
#         self._update_params = rospy.ServiceProxy('update_params', UpdateParams)
#
#         rospy.loginfo("waiting for emergency service")
#         rospy.wait_for_service('emergency')
#         rospy.loginfo("found emergency service")
#         self._emergency = rospy.ServiceProxy('emergency', Empty)
#
#         if use_controller:
#             rospy.loginfo("waiting for land service")
#             rospy.wait_for_service('land')
#             rospy.loginfo("found land service")
#             self._land = rospy.ServiceProxy('land', Empty)
#
#             rospy.loginfo("waiting for takeoff service")
#             rospy.wait_for_service('takeoff')
#             rospy.loginfo("found takeoff service")
#             self._takeoff = rospy.ServiceProxy('takeoff', Empty)
#         else:
#             self._land = None
#             self._takeoff = None
#
#         # subscribe to the joystick at the end to make sure that all required
#         # services were found
#         self._buttons = None
#         rospy.Subscriber(joy_topic, Joy, self._joyChanged)
#
#     def _joyChanged(self, data):
#         for i in range(0, len(data.buttons)):
#             if self._buttons == None or data.buttons[i] != self._buttons[i]:
#                 if i == 0 and data.buttons[i] == 1 and self._land != None:
#                     self._land()
#                 if i == 1 and data.buttons[i] == 1:
#                     self._emergency()
#                 if i == 2 and data.buttons[i] == 1 and self._takeoff != None:
#                     self._takeoff()
#                 if i == 4 and data.buttons[i] == 1:
#                     value = int(rospy.get_param("ring/headlightEnable"))
#                     if value == 0:
#                         rospy.set_param("ring/headlightEnable", 1)
#                     else:
#                         rospy.set_param("ring/headlightEnable", 0)
#                     self._update_params(["ring/headlightEnable"])
#                     print(not value)
#
#         self._buttons = data.buttons
#
# if __name__ == '__main__':
#     rospy.init_node('crazyflie_demo_controller', anonymous=True)
#     use_controller = rospy.get_param("~use_crazyflie_controller", False)
#     joy_topic = rospy.get_param("~joy_topic", "joy")
#     controller = Controller(use_controller, joy_topic)
#     rospy.spin()


#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty

class Controller():
    def __init__(self, use_controller, joy_topic):
        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

        rospy.set_param("commander/enHighLevel", 1)
        rospy.set_param("stabilizer/estimator", 2)  # Use EKF
        rospy.set_param("stabilizer/controller", 2)  # Use mellinger controller
        rospy.set_param("kalman/resetEstimation", 1)  # reset kalman

        # Mellinger control parameters
        rospy.set_param("ctrlMel/mass", 0.130)
        # rospy.set_param("ctrlMel/massThrust", 27500)
        rospy.set_param("ctrlMel/i_range_z", 4.0)
        rospy.set_param("ctrlMel/i_range_xy", 4.0)

        rospy.set_param("ctrlMel/kR_xy", 50000.0)
        rospy.set_param("ctrlMel/kw_xy", 16000.0)
        rospy.set_param("ctrlMel/kR_z", 60000.0)
        rospy.set_param("ctrlMel/kw_z", 12000.0)

        rospy.set_param("ctrlMel/kd_omega_rp", 200.0)

        rospy.set_param("ctrlMel/kp_xy", 1.5)
        rospy.set_param("ctrlMel/kd_xy", 0.8)
        rospy.set_param("ctrlMel/ki_xy", 0.1)

        rospy.set_param("ctrlMel/kp_z", 2.5)
        rospy.set_param("ctrlMel/kd_z", 0.5)
        rospy.set_param("ctrlMel/ki_z", 0.15)

        rospy.set_param("ctrlMel/ki_m_xy", 0.0)
        rospy.set_param("ctrlMel/ki_m_z", 500.0)

        # rospy.set_param("motorPowerSet/enable", False)
        # self._update_params(["motorPowerSet/enable"])
        # rospy.set_param("motorPowerSet/m1", 65535)
        # rospy.set_param("motorPowerSet/m2", 65535)
        # rospy.set_param("motorPowerSet/m3", 65535)
        # rospy.set_param("motorPowerSet/m4", 65535)
        # self._update_params(["motorPowerSet/m1"])
        # self._update_params(["motorPowerSet/m2"])
        # self._update_params(["motorPowerSet/m3"])
        # self._update_params(["motorPowerSet/m4"])

        rospy.loginfo("waiting for emergency service")
        rospy.wait_for_service('emergency')
        rospy.loginfo("found emergency service")

        self._emergency = rospy.ServiceProxy('emergency', Empty)

        if use_controller:
            rospy.loginfo("waiting for land service")
            rospy.wait_for_service('land')
            rospy.loginfo("found land service")
            self._land = rospy.ServiceProxy('land', Empty)

            rospy.loginfo("waiting for takeoff service")
            rospy.wait_for_service('takeoff')
            rospy.loginfo("found takeoff service")
            self._takeoff = rospy.ServiceProxy('takeoff', Empty)
        else:
            self._land = None
            self._takeoff = None

        # subscribe to the joystick at the end to make sure that all required
        # services were found
        self._buttons = None
        rospy.Subscriber(joy_topic, Joy, self._joyChanged)

    def _joyChanged(self, data):
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]:
                if i == 0 and data.buttons[i] == 1 and self._land != None:
                    self._land()
                if i == 1 and data.buttons[i] == 1:
                    self._emergency()
                if i == 2 and data.buttons[i] == 1 and self._takeoff != None:
                    self._takeoff()
                if i == 4 and data.buttons[i] == 1:
                    value = int(rospy.get_param("ring/headlightEnable"))



                    if value == 0:
                        rospy.set_param("ring/headlightEnable", 1)
                    else:
                        rospy.set_param("ring/headlightEnable", 0)

                    self._update_params(["ring/headlightEnable"])
                    print(not value)

        self._buttons = data.buttons

if __name__ == '__main__':
    rospy.init_node('crazyflie_demo_controller', anonymous=True)
    use_controller = rospy.get_param("~use_crazyflie_controller", False)
    joy_topic = rospy.get_param("~joy_topic", "joy")
    controller = Controller(use_controller, joy_topic)
    rospy.spin()
