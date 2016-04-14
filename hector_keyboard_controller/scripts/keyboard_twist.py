#!/usr/bin/env python

import pygame
import rospy
from geometry_msgs.msg import Twist

from keyboard_base import BaseKeyboardController


class KeyboardController(BaseKeyboardController):
    def _rospy_init(self, rate=10):
        BaseKeyboardController._rospy_init(self, rate)
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.msg = Twist()

    def _event(self, key_events):
        dx, dy, dz = 0.1, 0.1, 0.1
        dax, day, daz = 0.01, 0.01, 0.01

        if key_events[pygame.K_UP]:
            self.msg.linear.z += dz
        elif key_events[pygame.K_DOWN]:
            self.msg.linear.z -= dz

        if key_events[pygame.K_w]:
            self.msg.linear.y += dy
        elif key_events[pygame.K_s]:
            self.msg.linear.y -= dy

        if key_events[pygame.K_d]:
            self.msg.linear.x += dx
        elif key_events[pygame.K_a]:
            self.msg.linear.x -= dx

        if key_events[pygame.K_i]:
            self.msg.angular.z += daz
        elif key_events[pygame.K_k]:
            self.msg.angular.z -= daz

        if key_events[pygame.K_z]:
            self.msg.angular.x += dax
        elif key_events[pygame.K_x]:
            self.msg.angular.x -= dax

        if key_events[pygame.K_c]:
            self.msg.angular.y += day
        elif key_events[pygame.K_v]:
            self.msg.angular.y -= day

    def _draw(self):
        BaseKeyboardController._draw(self)
        width = float(self.background.get_rect().width)

        self._draw_x = width / 2
        self._draw_text("Keyboard Twist controller", 40)

        old_draw_y = self._draw_y
        self._draw_x = width / 4
        self._draw_text("Linear vel", 30)
        self._draw_text("X (d/a) : %.3f" % self.msg.linear.x)
        self._draw_text("Y (w/s) : %.3f" % self.msg.linear.y)
        self._draw_text("Z (up/down) : %.3f" % self.msg.linear.z)

        self._draw_y = old_draw_y # On the side
        self._draw_x = width * 3 / 4
        self._draw_text("Angular vel", 30)
        self._draw_text("X (z/x): %.3f" % self.msg.angular.x)
        self._draw_text("Y (c/v): %.3f" % self.msg.angular.y)
        self._draw_text("Z (i/k) : %.3f" % self.msg.angular.z)


if __name__ == '__main__':
    try:
        keyboard = KeyboardController()
        keyboard.run()
    except rospy.ROSInterruptException:
        print("Closed the program due to ROS Interrupt exception")
    except KeyboardInterrupt:
        print("Closed the program due to Keyboard Interrupt exception")

