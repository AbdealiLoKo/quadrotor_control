#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
import pygame, sys

class SarsaController:
    def __init__(self):
        self._rospy_init()

        num_states = 6
        num_actions = 2

        self.state = 0
        self.reward = 0
        self.action = 0

        self.old_state = None
        self.old_action = None

        self.stepsize = "harmonic"
        self.discounting_factor = 0.5

        self.weights = numpy.zeros([num_states, num_actions])
        self.num_action = numpy.zeros([num_actions,])

    def _rospy_init(self, rate=10):
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('controller', anonymous=True)
        self.rate = rospy.Rate(rate) # in Hz
        self.msg = Twist()

    def run(self):
        while not rospy.is_shutdown():
            if self.action == 0:
                self.msg.linear.z = 1;
            elif self.action == 1:
                self.msg.linear.z = -1;

            rospy.loginfo(self.msg)
            self.publisher.publish(self.msg)
            self._sleep()

    def _event(self, key_events):
        if self.type == "cartesian":
            self._cartesian_event(key_events)
        else:
            print("Unknown keyboard controlle rtype:", self.type)

    def _sleep(self):
        self.rate.sleep()

if __name__ == '__main__':
    try:
        keyboard = KeyboardController(type="cartesian")
        keyboard.run()
    except rospy.ROSInterruptException:
        print("Closed the program due to ROS Interrupt exception")
    except KeyboardInterrupt:
        print("Closed the program due to Keyboard Interrupt exception")

