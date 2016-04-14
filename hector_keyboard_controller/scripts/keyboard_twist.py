#!/usr/bin/env python

import pygame
import rospy
from geometry_msgs.msg import Twist

from keyboard_base import BaseKeyboardController


class KeyboardTwistController(BaseKeyboardController):
    def rospy_init(self, rate=10):
        BaseKeyboardController.rospy_init(self, rate)
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.msg = Twist()

    def event_handler(self, key_events):
        dx = dy = dz = 0.001
        dax = day = daz = 0.0001

        twist_events = { # x, y, z, roll, pitch, yaw
            pygame.K_e: [dx, 0, 0, 0, 0, 0],
            pygame.K_q: [-dx, 0, 0, 0, 0, 0],
            pygame.K_d: [0, dy, 0, 0, 0, 0],
            pygame.K_a: [0, -dy, 0, 0, 0, 0],
            pygame.K_w: [0, 0, dz, 0, 0, 0],
            pygame.K_s: [0, 0, -dz, 0, 0, 0],

            pygame.K_o: [0, 0, 0, dax, 0, 0],
            pygame.K_u: [0, 0, 0, -dax, 0, 0],
            pygame.K_l: [0, 0, 0, 0, day, 0],
            pygame.K_j: [0, 0, 0, 0, -day, 0],
            pygame.K_i: [0, 0, 0, 0, 0, daz],
            pygame.K_k: [0, 0, 0, 0, 0, -daz],
        }
        if key_events[pygame.K_0]:
            assert self.gz_reset()
            self.msg.linear.z = 0
            self.msg.linear.y = 0
            self.msg.linear.x = 0
            self.msg.angular.x = 0
            self.msg.angular.y = 0
            self.msg.angular.z = 0
        else:
            for key in twist_events:
                if key_events[key]:
                    x, y, z, ax, ay, az = twist_events[key]
                    self.msg.linear.x += x
                    self.msg.linear.y += y
                    self.msg.linear.z += z
                    self.msg.angular.x += ax
                    self.msg.angular.y += ay
                    self.msg.angular.z += az

    def draw(self):
        BaseKeyboardController.draw(self)
        width = float(self.background.get_rect().width)

        self._draw_x = width / 2
        self._draw_text("Keyboard Twist controller", 40)

        old_draw_y = self._draw_y
        self._draw_x = width / 4
        self._draw_text("Linear vel", 30)
        self._draw_text("X (q/e) : %.3f" % self.msg.linear.x)
        self._draw_text("Y (a/d) : %.3f" % self.msg.linear.y)
        self._draw_text("Z (w/s) : %.3f" % self.msg.linear.z)

        self._draw_y = old_draw_y # On the side
        self._draw_x = width * 3 / 4
        self._draw_text("Angular vel", 30)
        self._draw_text("X (u/o): %.5f" % self.msg.angular.x)
        self._draw_text("Y (j/l): %.5f" % self.msg.angular.y)
        self._draw_text("Z (i/k) : %.5f" % self.msg.angular.z)


if __name__ == '__main__':
    try:
        keyboard = KeyboardTwistController()
        keyboard.run()
    except rospy.ROSInterruptException:
        print("Closed the program due to ROS Interrupt exception")
    except KeyboardInterrupt:
        print("Closed the program due to Keyboard Interrupt exception")

