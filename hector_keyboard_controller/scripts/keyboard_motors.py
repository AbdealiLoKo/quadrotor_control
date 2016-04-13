#!/usr/bin/env python

import rospy, tf
import time, sys, math
from geometry_msgs.msg import Twist, Pose, Quaternion
from hector_uav_msgs.msg import MotorPWM
from base_controller import BaseController

from gazebo_msgs.msg import ModelStates
import pygame

class KeyboardControllerMotors(BaseController):
    def _rospy_init(self, rate=100):
        self.publisher = rospy.Publisher('motor_pwm', MotorPWM, queue_size=100)
        self.model_states = rospy.Subscriber(
            '/gazebo/model_states',
            ModelStates,
            self.get_modelstates,
            queue_size=10)
        rospy.init_node('keyboard_controller', anonymous=True)

        self.quad_pose = Pose()
        self.quad_twist = Twist()
        self.rate = rospy.Rate(rate) # in Hz
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
        self.background = pygame.Surface(self.screen.get_size())
        self.background = self.background.convert()
        self.background.fill((250, 250, 250))

        width = float(self.background.get_rect().width)

        self._draw_y = 20
        self._draw_x = width / 2
        self._draw_text("Keyboard controller", 40)

        old_draw_y = self._draw_y
        self._draw_x = width / 4
        self._draw_text("Motor", 30)
        self._draw_text("Motor 1: %.3f" % self.msg.pwm[0])
        self._draw_text("Motor 2: %.3f" % self.msg.pwm[1])
        self._draw_text("Motor 3: %.3f" % self.msg.pwm[2])
        self._draw_text("Motor 4: %.3f" % self.msg.pwm[3])

        self._draw_x = width / 2
        self._draw_text("Quadrotor state", 40)

        old_draw_y = self._draw_y
        self._draw_x = width / 4
        self._draw_text("Position", 30)
        self._draw_text("X : %.3f" % self.quad_pose.position.x)
        self._draw_text("Y : %.3f" % self.quad_pose.position.y)
        self._draw_text("Z : %.3f" % self.quad_pose.position.z)

        self._draw_text("Linear velocity", 30)
        self._draw_text("X : %.3f" % self.quad_twist.linear.x)
        self._draw_text("Y : %.3f" % self.quad_twist.linear.y)
        self._draw_text("Z : %.3f" % self.quad_twist.linear.z)

        self._draw_y = old_draw_y # On the side
        self._draw_x = width * 3 / 4
        self._draw_text("Orientation", 30)

        old_draw_y = self._draw_y
        self._draw_x = width * 5 / 8
        yaw, pitch, roll  = tf.transformations.euler_from_quaternion(
            self.msg_to_quaternion(self.quad_pose.orientation))
        self._draw_text("R : %.3f" % math.degrees(roll))
        self._draw_text("P : %.3f" % math.degrees(pitch))
        self._draw_text("Y : %.3f" % math.degrees(yaw))

        self._draw_y = old_draw_y
        self._draw_x = width * 7 / 8
        self._draw_text("X : %.3f" % self.quad_pose.orientation.x)
        self._draw_text("Y : %.3f" % self.quad_pose.orientation.y)
        self._draw_text("Z : %.3f" % self.quad_pose.orientation.z)
        self._draw_text("W : %.3f" % self.quad_pose.orientation.w)

        self._draw_x = width * 3 / 4
        self._draw_text("Angular velocity", 30)
        self._draw_text("X : %.3f" % self.quad_twist.angular.x)
        self._draw_text("Y : %.3f" % self.quad_twist.angular.y)
        self._draw_text("Z : %.3f" % self.quad_twist.angular.z)

        self.screen.blit(self.background, (0, 0))
        pygame.display.flip()

if __name__ == '__main__':
    try:
        keyboard = KeyboardControllerMotors()
        keyboard.run()
    except rospy.ROSInterruptException:
        print("Closed the program due to ROS Interrupt exception")
    except KeyboardInterrupt:
        print("Closed the program due to Keyboard Interrupt exception")
