#!/usr/bin/env python

import rospy
import time
from geometry_msgs.msg import Twist
import pygame, sys

class KeyboardController:
    def __init__(self, type="cartesian"):
        self.type = type
        self._gui_init()
        self._rospy_init()

    def _gui_init(self):
        pygame.init()
        self.screen = pygame.display.set_mode([300, 300])
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
        rospy.init_node('controller', anonymous=True)
        self.rate = rospy.Rate(rate) # in Hz
        self.msg = Twist()

    def run(self):
        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            self._event(pygame.key.get_pressed())
            self._draw()

            rospy.loginfo(self.msg)
            self.publisher.publish(self.msg)
            self._sleep()

    def _event(self, key_events):
        if self.type == "cartesian":
            self._cartesian_event(key_events)
        else:
            print("Unknown keyboard controlle rtype:", self.type)

    def _cartesian_event(self, key_events):
        if key_events[pygame.K_UP]:
            self.msg.linear.z = 1;
        elif key_events[pygame.K_DOWN]:
            self.msg.linear.z = -1;

        if key_events[pygame.K_w]:
            self.msg.linear.y = 1;
        elif key_events[pygame.K_s]:
            self.msg.linear.y = -1;

        if key_events[pygame.K_d]:
            self.msg.linear.x = 1;
        elif key_events[pygame.K_a]:
            self.msg.linear.x = -1;

    def _draw(self):
        background = pygame.Surface(self.screen.get_size())
        background = background.convert()
        background.fill((250, 250, 250))

        centery = 20
        font = pygame.font.Font(None, 36)
        text = font.render("Keyboard controller", 1, (10, 10, 10))
        textpos = text.get_rect()
        textpos.centerx = background.get_rect().centerx
        background.blit(text, textpos)

        centery += 20 + 15 + 30
        font = pygame.font.Font(None, 30)
        text = font.render("Linear vel", 1, (10, 10, 10))
        textpos = text.get_rect()
        textpos.centerx = background.get_rect().centerx
        textpos.centery = centery
        background.blit(text, textpos)

        centery += 15 + 10
        font = pygame.font.Font(None, 20)
        text = font.render("X : " + str(self.msg.linear.x), 1, (10, 10, 10))
        textpos = text.get_rect()
        textpos.centerx = background.get_rect().centerx
        textpos.centery = centery
        background.blit(text, textpos)

        centery += 10 + 10
        text = font.render("Y : " + str(self.msg.linear.y), 1, (10, 10, 10))
        textpos = text.get_rect()
        textpos.centerx = background.get_rect().centerx
        textpos.centery = centery
        background.blit(text, textpos)

        centery += 10 + 10
        text = font.render("Z : " + str(self.msg.linear.z), 1, (10, 10, 10))
        textpos = text.get_rect()
        textpos.centerx = background.get_rect().centerx
        textpos.centery = centery
        background.blit(text, textpos)

        centery += 10 + 15 + 30
        font = pygame.font.Font(None, 30)
        text = font.render("Angular vel", 1, (10, 10, 10))
        textpos = text.get_rect()
        textpos.centerx = background.get_rect().centerx
        textpos.centery = centery
        background.blit(text, textpos)

        centery += 15 + 10
        font = pygame.font.Font(None, 20)
        text = font.render("W1 : " + str(self.msg.angular.x), 1, (10, 10, 10))
        textpos = text.get_rect()
        textpos.centerx = background.get_rect().centerx
        textpos.centery = centery
        background.blit(text, textpos)

        centery += 10 + 10
        text = font.render("W2 : " + str(self.msg.angular.y), 1, (10, 10, 10))
        textpos = text.get_rect()
        textpos.centerx = background.get_rect().centerx
        textpos.centery = centery
        background.blit(text, textpos)

        centery += 10 + 10
        text = font.render("W3 : " + str(self.msg.angular.z), 1, (10, 10, 10))
        textpos = text.get_rect()
        textpos.centerx = background.get_rect().centerx
        textpos.centery = centery
        background.blit(text, textpos)

        self.screen.blit(background, (0, 0))
        pygame.display.flip()

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

