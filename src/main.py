#!/usr/bin/env python

import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from smach import State, StateMachine
from sensor_msgs.msg import Joy
import smach_ros
from dynamic_reconfigure.server import Server
from demo3.cfg import Demo3Config

START = True
RED_VISIBLE = False


class WaitForButton(State):
    def __init__(self):
        State.__init__(self, outcomes=["pressed", "exit"])
        self.rate = rospy.Rate(10)

    def execute(self, userdata):
        global START
        while not START and not rospy.is_shutdown():
            self.rate.sleep()
        if rospy.is_shutdown():
            return "exit"
        else:
            return "pressed"


class FollowWhite(State):
    def __init__(self):
        State.__init__(self, outcomes=["see_red", "exit"])

        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
                                          Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',
                                           Twist, queue_size=1)

        srv = Server(Demo3Config, self.dr_callback)

        self.twist = Twist()
        self.found_red = False
        self.cx = None
        self.cy = None
        self.w = None
        self.Kp = 1.0 / 500.0
        self.Kd = 1.0 / 1000.0
        self.Ki = 0
        self.dt = 1.0 / 20
        self.linear_vel = 0.2

        self.white_max_h = 250
        self.white_max_s = 60
        self.white_max_v = 256

        self.white_min_h = 0
        self.white_min_s = 0
        self.white_min_v = 230

        self.red_max_h = 360
        self.red_max_s = 256
        self.red_max_v = 225

        self.red_min_h = 150
        self.red_min_s = 150
        self.red_min_v = 80

    def image_callback(self, msg):
        global RED_VISIBLE

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hsv2 = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # lower_white = numpy.array([0,  0,  80])
        # upper_white = numpy.array([360, 15, 170])
        lower_red = numpy.array([self.red_min_h,  self.red_min_s,  self.red_min_v])
        upper_red = numpy.array([self.red_max_h, self.red_max_s, self.red_max_v])

        lower_white = numpy.array([self.white_min_h,  self.white_min_s,  self.white_min_v])
        upper_white = numpy.array([self.white_max_h, self.white_max_s, self.white_max_v])
        # lower_red = numpy.array([300, 94, 30])
        # upper_red = numpy.array([360, 256, 256])
        mask = cv2.inRange(hsv, lower_white, upper_white)
        mask_red = cv2.inRange(hsv2, lower_red, upper_red)

        h, w, d = image.shape
        self.w = w
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        # cv2.imshow("window", mask)
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            self.cx = cx
            self.cy = cy
            cv2.circle(image, (cx, cy), 20, (0, 0, 255), -1)

        cv2.imshow("window", mask_red)
        mask_red[0:search_top, 0:w] = 0
        mask_red[search_bot:h, 0:w] = 0
        M_red = cv2.moments(mask_red)
        if M_red['m00'] > 0:
            print("?????", "found red")
            self.found_red = True
            cx_red = int(M_red['m10']/M_red['m00'])
            cy_red = int(M_red['m01']/M_red['m00'])
            cv2.circle(image, (cx, cy), 20, (0, 0, 205), -1)
        else:
            self.found_red = False
            RED_VISIBLE = False

        cv2.waitKey(3)

    def dr_callback(self, config, level):
        self.Kp = config["Kp"]
        self.Kd = config["Kd"]
        self.Ki = config["Ki"]
        self.linear_vel = config["linear_vel"]

        self.white_max_h = config["white_max_h"]
        self.white_max_s = config["white_max_s"]
        self.white_max_v = config["white_max_v"]

        self.white_min_h = config["white_min_h"]
        self.white_min_s = config["white_min_s"]
        self.white_min_v = config["white_min_v"]

        self.red_max_h = config["red_max_h"]
        self.red_max_s = config["red_max_s"]
        self.red_max_v = config["red_max_v"]

        self.red_min_h = config["red_min_h"]
        self.red_min_s = config["red_min_s"]
        self.red_min_v = config["red_min_v"]

        return config

    def execute(self, userdata):
        global RED_VISIBLE
        previous_error = 0
        integral = 0
        sleep_duration = rospy.Duration(self.dt, 0)

        while not rospy.is_shutdown() and START:
            if self.found_red and not RED_VISIBLE:
                break
            # BEGIN CONTROL
            if self.cx is not None and self.w is not None:
                error = self.cx - self.w/2
                integral += error * self.dt
                derivative = (error - previous_error) / self.dt

                self.twist.linear.x = self.linear_vel
                self.twist.angular.z = - \
                    (self.Kp * float(error) + self.Kd *
                     derivative + self.Ki * integral)
                self.cmd_vel_pub.publish(self.twist)

                previous_error = error

                rospy.sleep(sleep_duration)
            # END CONTROL
        if not START:
            return "exit"
        if self.found_red:
            print("??????", RED_VISIBLE)
            return "see_red"


class StopAtRed(State):
    def __init__(self):
        State.__init__(self, outcomes=["time_out", "exit"])

    def execute(self, userdata):
        global RED_VISIBLE
        RED_VISIBLE = True
        rospy.sleep(rospy.Duration(2))
        print("sleep is done, I should be moving now")
        if not START:
            return "exit"
        return "time_out"


def joy_callback(msg):
    global START

    if msg.buttons[0] == 1:  # button A
        START = True
    elif msg.buttons[1] == 1:  # button B
        START = False


if __name__ == "__main__":
    rospy.init_node('demo3')
    rospy.Subscriber("/joy", Joy, callback=joy_callback)

    sm = StateMachine(outcomes=['success', 'failure'])
    with sm:
        StateMachine.add("Wait", WaitForButton(),
                         transitions={'pressed': 'Follow', 'exit': 'failure'})
        StateMachine.add("Follow", FollowWhite(), transitions={
                         "see_red": "Stop", "exit": "Wait"})
        StateMachine.add("Stop", StopAtRed(), transitions={
                         "time_out": "Follow", "exit": "Wait"})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
