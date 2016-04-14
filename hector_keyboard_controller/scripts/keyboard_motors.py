#!/usr/bin/env python

import pygame
import rospy

from keyboard_base import BaseKeyboardController
from hector_uav_msgs.msg import MotorPWM


class KeyboardControllerMotors(BaseKeyboardController):
    def rospy_init(self, rate=100):
        BaseKeyboardController.rospy_init(self, rate)
        self.publisher = rospy.Publisher('motor_pwm', MotorPWM, queue_size=100)
        self.msg = MotorPWM()
        self.msg.pwm = [50] * 4

    def event_handler(self, key_events):
        dm1 = dm2 = dm3 = dm4 = 0.1
        motor_events = {
            pygame.K_q: [dm1, 0, 0, 0],
            pygame.K_a: [-dm1, 0, 0, 0],
            pygame.K_w: [0, dm2, 0, 0],
            pygame.K_s: [0, -dm2, 0, 0],
            pygame.K_e: [0, 0, dm3, 0],
            pygame.K_d: [0, 0, -dm3, 0],
            pygame.K_r: [0, 0, 0, dm4],
            pygame.K_f: [0, 0, 0, -dm4],
        }

        if key_events[pygame.K_0]:
            assert self.gz_reset()
            self.msg.pwm = [50] * 4
        else:
            for key in motor_events:
                if key_events[key]:
                    self.msg.pwm = [
                        max(min(i + k, 255), 0)
                        for i, k in zip(motor_events[key], self.msg.pwm)]

    def draw(self):
        BaseKeyboardController.draw(self)
        width = float(self.background.get_rect().width)

        self._draw_x = width / 2
        self._draw_text("Keyboard Motor controller", 40)

        self._draw_text("Motor 1 (q/a): %.3f" % self.msg.pwm[0])
        self._draw_text("Motor 2 (w/s): %.3f" % self.msg.pwm[1])
        self._draw_text("Motor 3 (e/d): %.3f" % self.msg.pwm[2])
        self._draw_text("Motor 4 (r/f): %.3f" % self.msg.pwm[3])

if __name__ == '__main__':
    try:
        keyboard = KeyboardControllerMotors()
        keyboard.run()
    except rospy.ROSInterruptException:
        print("Closed the program due to ROS Interrupt exception")
    except KeyboardInterrupt:
        print("Closed the program due to Keyboard Interrupt exception")
