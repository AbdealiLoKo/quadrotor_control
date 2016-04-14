#!/usr/bin/env python

import pygame
import rospy

from keyboard_base import BaseKeyboardController
from hector_uav_msgs.msg import MotorPWM


class KeyboardControllerMotors(BaseKeyboardController):
    def _rospy_init(self, rate=100):
        BaseKeyboardController._rospy_init(self, rate)
        self.publisher = rospy.Publisher('motor_pwm', MotorPWM, queue_size=100)
        self.msg = MotorPWM()
        self.msg.pwm = [200]*4

    def _event(self, key_events):
        dx = 0.1
        dx1, dx2, dx3, dx4 = 1, 1, 1, 1

        if key_events[pygame.K_q]:
            self.msg.pwm[0] += dx
        elif key_events[pygame.K_a]:
            self.msg.pwm[0] -= dx

        if key_events[pygame.K_w]:
            self.msg.pwm[1] += dx
        elif key_events[pygame.K_s]:
            self.msg.pwm[1] -= dx

        if key_events[pygame.K_e]:
            self.msg.pwm[2] += dx
        elif key_events[pygame.K_d]:
            self.msg.pwm[2] -= dx

        if key_events[pygame.K_r]:
            self.msg.pwm[3] += dx
        elif key_events[pygame.K_f]:
            self.msg.pwm[3] -= dx

    def _draw(self):
        BaseKeyboardController._draw(self)
        width = float(self.background.get_rect().width)

        self._draw_x = width / 2
        self._draw_text("Keyboard Motor controller", 40)

        self._draw_text("Motor 1: %.3f" % self.msg.pwm[0])
        self._draw_text("Motor 2: %.3f" % self.msg.pwm[1])
        self._draw_text("Motor 3: %.3f" % self.msg.pwm[2])
        self._draw_text("Motor 4: %.3f" % self.msg.pwm[3])

if __name__ == '__main__':
    try:
        keyboard = KeyboardControllerMotors()
        keyboard.run()
    except rospy.ROSInterruptException:
        print("Closed the program due to ROS Interrupt exception")
    except KeyboardInterrupt:
        print("Closed the program due to Keyboard Interrupt exception")
