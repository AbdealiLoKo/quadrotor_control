import time, sys, math
import pygame

import rospy, tf
from geometry_msgs.msg import Twist, Pose, Quaternion
from gazebo_msgs.msg import ModelStates
from std_srvs.srv import Empty


class BaseKeyboardController:
    def __init__(self):
        self.gui_init()
        self.rospy_init()
        rospy.loginfo(rospy.get_name() + ' -- Initialization complete')

    def gui_init(self):
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

    def rospy_init(self, rate=100):
        rospy.init_node('keyboard_controller', anonymous=True)

        self.model_states = rospy.Subscriber(
            '/gazebo/model_states',
            ModelStates,
            self.get_modelstates,
            queue_size=10)
        rospy.wait_for_service("/gazebo/reset_world")
        self.gz_reset = rospy.ServiceProxy("/gazebo/reset_world", Empty)

        self.quad_pose = Pose()
        self.quad_twist = Twist()
        self.rate = rospy.Rate(rate) # in Hz

        # Needs to be overridden by child class
        self.msg = None

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
        pygame.event.post(pygame.event.Event(pygame.KEYUP, key=pygame.K_0))
        while not rospy.is_shutdown():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

            self.event_handler(pygame.key.get_pressed())
            self.draw_now()

            self.publisher.publish(self.msg)
            self.sleep()

    def event_handler(self, key_events):
        raise NotImplementedError

    def draw_now(self):
        self.draw()
        self.screen.blit(self.background, (0, 0))
        pygame.display.flip()

    def draw(self):
        self.background = pygame.Surface(self.screen.get_size())
        self.background = self.background.convert()
        self.background.fill((250, 250, 250))

        width = float(self.background.get_rect().width)
        self._draw_y = 20
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

    def sleep(self):
        self.rate.sleep()
