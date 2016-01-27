#!/usr/bin/env python

import rospy
import time, sys, math
from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.msg import ModelStates
import pygame


class KeyboardController:
    def __init__(self):
        self._gui_init()
        self._rospy_init()
        rospy.loginfo(rospy.get_name() + ' -- Initialization complete')

    def _gui_init(self):
        pygame.init()
        self.screen = pygame.display.set_mode([400, 600])
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

    def _rospy_init(self, rate=10):
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.model_states = rospy.Subscriber(
            '/gazebo/model_states',
            ModelStates,
            self.get_modelstates,
            queue_size=10)
        rospy.init_node('keyboard_controller', anonymous=True)

        self.quad_pose = Pose()
        self.quad_twist = Twist()
        self.rate = rospy.Rate(rate) # in Hz
        self.msg = Twist()

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
        dx, dy, dz = 0.1, 0.1, 0.1

        if key_events[pygame.K_UP]:
            self.msg.linear.z += dz;
        elif key_events[pygame.K_DOWN]:
            self.msg.linear.z -= dz;

        if key_events[pygame.K_w]:
            self.msg.linear.y += dy;
        elif key_events[pygame.K_s]:
            self.msg.linear.y -= dy;

        if key_events[pygame.K_d]:
            self.msg.linear.x += dx;
        elif key_events[pygame.K_a]:
            self.msg.linear.x -= dx;

    def _draw(self):
        self.background = pygame.Surface(self.screen.get_size())
        self.background = self.background.convert()
        self.background.fill((250, 250, 250))

        self._draw_y = 20
        self._draw_x = self.background.get_rect().centerx

        self._draw_text("Keyboard controller", 40)
        self._draw_text("Linear vel", 30)
        self._draw_text("X (d/a) : %.3f" % self.msg.linear.x)
        self._draw_text("Y (w/s) : %.3f" % self.msg.linear.y)
        self._draw_text("Z (up/down) : %.3f" % self.msg.linear.z)
        self._draw_text("Angular vel", 30)
        self._draw_text("X : %.3f" % self.msg.angular.x)
        self._draw_text("Y : %.3f" % self.msg.angular.y)
        self._draw_text("Z : %.3f" % self.msg.angular.z)

        self._draw_text("Quadrotor state", 40)

        old_draw_y = self._draw_y
        self._draw_x = self.background.get_rect().centerx / 2

        self._draw_text("Position", 30)
        self._draw_text("X : %.3f" % self.quad_pose.position.x)
        self._draw_text("Y : %.3f" % self.quad_pose.position.y)
        self._draw_text("Z : %.3f" % self.quad_pose.position.z)
        self._draw_text("Orientation", 30)
        self._draw_text("X : %.3f" % self.quad_pose.orientation.x)
        self._draw_text("Y : %.3f" % self.quad_pose.orientation.y)
        self._draw_text("Z : %.3f" % self.quad_pose.orientation.z)
        self._draw_text("W : %.3f" % self.quad_pose.orientation.w)

        self._draw_y = old_draw_y
        self._draw_x = self._draw_x + self.background.get_rect().centerx
        self._draw_text("Linear velocity", 30)
        self._draw_text("X : %.3f" % self.quad_twist.linear.x)
        self._draw_text("Y : %.3f" % self.quad_twist.linear.y)
        self._draw_text("Z : %.3f" % self.quad_twist.linear.z)
        self._draw_text("Angular velocity", 30)
        self._draw_text("X : %.3f" % self.quad_twist.angular.x)
        self._draw_text("Y : %.3f" % self.quad_twist.angular.y)
        self._draw_text("Z : %.3f" % self.quad_twist.angular.z)

        self.screen.blit(self.background, (0, 0))
        pygame.display.flip()

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

