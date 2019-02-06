#!/usr/bin/env python

import rospy
from smach import State, StateMachine
from sensor_msgs.msg import Joy
import smach_ros

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

    def execute(self, userdata):
        pass


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
                         "seed_red": "Stop", "exit": "Wait"})
        StateMachine.add("Stop", StopAtRed(), transitions={
                         "time_out": "Follow", "exit": "Wait"})

    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
    outcome = sm.execute()
    sis.stop()
