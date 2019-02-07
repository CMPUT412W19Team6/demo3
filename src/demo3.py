#!/usr/bin/env python

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from smach import State, StateMachine
from sensor_msgs.msg import Joy
import smach_ros
# from dynamic_reconfigure.server import Server
# from demo3.cfg import Demo3Config

START = False


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
        
        # srv = Server(FollowbotConfig, self.dr_callback)

        self.twist = Twist()
        self.start = False
        self.cx = None
        self.cy = None
        self.w = None
        self.Kp = 1.0 / 1500.0
        self.Kd = 1.0 / 1000.0
        self.Ki = 0
        self.dt = 1.0 / 20
        self.linear_vel = 0.2
    
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([ 10,  10,  10])
        upper_yellow = numpy.array([255, 255, 250])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        h, w, d = image.shape
        self.w = w
        search_top = 3*h/4
        search_bot = 3*h/4 + 20
        mask[0:search_top, 0:w] = 0
        mask[search_bot:h, 0:w] = 0
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            self.cx = cx
            self.cy = cy
            cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
        
        cv2.imshow("window", image)
        cv2.waitKey(3)
    
    def dr_callback(self, config, level):
        self.Kp = config["Kp"]
        self.Kd = config["Kd"]
        self.Ki = config["Ki"]
        self.linear_vel = config["linear_vel"]

        return config

    def execute(self, userdata):
        previous_error = 0
        integral = 0
        sleep_duration = rospy.Duration(self.dt, 0)

        while not rospy.is_shutdown() and START:
            # BEGIN CONTROL
            if self.cx is not None and self.w is not None:
                error = self.cx - self.w/2
                integral += error * self.dt
                derivative = (error - previous_error) / self.dt

                self.twist.linear.x = self.linear_vel
                self.twist.angular.z = -(self.Kp * float(error) + self.Kd * derivative + self.Ki * integral)
                self.cmd_vel_pub.publish(self.twist)

                previous_error = error

                rospy.sleep(sleep_duration)
            # END CONTROL
        if not START:
            return "exit"


class StopAtRed(State):
    def __init__(self):
        State.__init__(self, outcomes=["time_out", "exit"])

    def execute(self, userdata):
        pass


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
