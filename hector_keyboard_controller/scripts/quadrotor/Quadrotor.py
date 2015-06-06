from geometry_msgs.msg import Twist

class Quadrotor:
    """
    Holds all information about a quadrotor and can control the quadrotor
    completely
    """
    def __init__(self):
        self.pos = [0, 0, 0]
        self.vel = [0, 0, 0]
        self.acc = [0, 0, 0]

        self.ang = [0, 0, 0]
        self.ang_vel = [0, 0, 0]
        self.ang_acc = [0, 0, 0]

        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    def send(self):
        """
        Makes a Twist message and sends it
        """
        msg = Twist()

        msg.linear.x = self.vel[0]
        msg.linear.y = self.vel[1]
        msg.linear.z = self.vel[2]
        msg.angular.x = self.ang_vel[0]
        msg.angular.y = self.ang_vel[1]
        msg.angular.z = self.ang_vel[2]

        self.publisher.publish(msg)