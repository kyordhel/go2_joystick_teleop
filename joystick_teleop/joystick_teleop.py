import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('joystick_teleop')
        self.__pubCmdVel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.__pubTrick  = self.create_publisher(String, '/go2_trick', 10)
        self.__sub = self.create_subscription(Joy,'/joy',
            self.joyCallback, 10)
    #end def

    def joyCallback(self, msgJoy):
        msgTrick = String()
        if msgJoy.buttons[0]:
            msgTrick.data = "stand"
            self.__pubTrick.publish(msgTrick)
            return
        elif msgJoy.buttons[1]:
            msgTrick.data = "sit"
            self.__pubTrick.publish(msgTrick)
            return
        elif msgJoy.buttons[2]:
            msgTrick.data = "lay"
            self.__pubTrick.publish(msgTrick)
            return

        leftStickX  = msgJoy.axes[0]
        leftStickY  = msgJoy.axes[1]
        rightStickX = msgJoy.axes[2]
        rightStickY = msgJoy.axes[3]

        lx, az = self.axis2lxaz(leftStickX, leftStickY)
        msgTwist = Twist()
        msgTwist.linear.x = lx
        msgTwist.linear.y = 0.0
        msgTwist.linear.z = 0.0
        msgTwist.angular.x = 0.0
        msgTwist.angular.y = 0.0
        msgTwist.angular.z = az
        self.__pubCmdVel.publish(msgTwist)

    #end def

    def axis2lxaz(self, x, y):
        magnitude = math.sqrt(x**2 + y**2)
        if magnitude < 0.1:
            return 0.0, 0.0
        lx = 0.5 * y
        az = 0.8 * x
        return lx, az
    #end def

    def xy2rtheta(self, x, y):
        r = math.sqrt(x**2 + y**2)
        theta = math.atan2(x, y)
        return r, theta
    #end def
#end class


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
