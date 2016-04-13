#!/usr/bin/env python

import rospy, tf
import time, sys, math
from geometry_msgs.msg import Twist, Pose, Quaternion
from hector_uav_msgs.msg import MotorPWM

from gazebo_msgs.msg import ModelStates
import pygame


class BaseController:
    def __init__(self):
        self._gui_init()
        self._rospy_init()
        rospy.loginfo(rospy.get_name() + ' -- Initialization complete')

    def _gui_init(self):
        pygame.init()
        self.screen = pygame.display.set_mode([400, 500])
        self.clock = pygame.time.Clock()

        background = pygame.Surface(self.screen.get_size())
        background = background.convert()
        background.fill((250, 250, 250))

        font = pygame.font.Font(None, 36)

        text = font.render("Keyboard controller", 1, (10, 10, 10))
        textpos = text.get_rect()
        textpos.centerx = background.get_rect().centerx
        background.blit(text, textpos)

        font = pygame.font.Font(None, 30)
        text = font.render("Not connected", 1, (200, 10, 10))
        textpos = text.get_rect()
        textpos.centerx = background.get_rect().centerx
        textpos.centery = background.get_rect().centery
        background.blit(text, textpos)

        self.screen.blit(background, (0, 0))
        pygame.display.flip()

    def _msg_init(self, rate=100):
        pass

    def _rospy_init(self, rate=100):
        self.model_states = rospy.Subscriber(
            '/gazebo/model_states',
            ModelStates,
            self.get_modelstates,
            queue_size=10)
        rospy.init_node('keyboard_controller', anonymous=True)

    def get_modelstates(self, data):
        quad_id = None
        for i, name in enumerate(data.name):
            if name == 'quadrotor':
                quad_id = i
                break
        if quad_id == None:
            return
        self.quad_pose = data.pose[quad_id]
        self.quad_twist = data.twist[quad_id]

    def run(self):
        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            self._event(pygame.key.get_pressed())
            self._draw()

            self.publisher.publish(self.msg)
            self._sleep()

    def _event(self, key_events):
        pass

    def _draw(self):
        pass

    def _draw_text(self, text, font_size=20, x=None, y=None):
        x = x or self._draw_x
        y = y or self._draw_y

        font = pygame.font.Font(None, font_size)
        text = font.render(text, 1, (10, 10, 10))
        textpos = text.get_rect()
        textpos.centerx = x
        textpos.centery = y
        self.background.blit(text, textpos)

        self._draw_y += font_size + 5

    @staticmethod
    def msg_to_quaternion(q):
        assert type(q) == Quaternion
        # Quaternions in TF are normally SXYZ form
        return [q.w, q.x, q.y, q.z]

    def _sleep(self):
        self.rate.sleep()

if __name__ == '__main__':
    try:
        keyboard = KeyboardController()
        keyboard.run()
    except rospy.ROSInterruptException:
        print("Closed the program due to ROS Interrupt exception")
    except KeyboardInterrupt:
        print("Closed the program due to Keyboard Interrupt exception")

