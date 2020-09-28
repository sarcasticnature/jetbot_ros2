from math import sin, cos, radians
import rclpy
from rclpy.node import Node

import qwiic_scmd
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')

        self.motor_driver = qwiic_scmd.QwiicScmd()
        self.left_motor = 0
        self.right_motor = 1
        self.all_stop()

        self.string_sub = self.create_subscription(
            String,
            'jetbot_motor_string',
            self.string_callback,
            10)
        
        self.twist_sub = self.create_subscription(
            Twist,
            'jetbot_motor_twist',
            self.twist_callback,
            10)


    # sets motor speed between [-1.0, 1.0]
    def set_speed(self, motor_ID, value):
        max_pwm = 115.0
        speed = int(value * max_pwm)
    
        if motor_ID == 0 or 1:
            self.motor_driver.set_drive(motor_ID, 0, speed)
        else:
            self.get_logger().error('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
            return
    
    # stops all motors
    def all_stop(self):
        self.motor_driver.set_drive(self.left_motor,0,0)
        self.motor_driver.set_drive(self.right_motor,0,0)
    
        self.motor_driver.disable()

    def string_callback(self, msg):
        if msg.data == "left":
            self.set_speed(self.left_motor,  -1.0)
            self.set_speed(self.right_motor,  1.0)
            self.motor_driver.enable()
        elif msg.data == "right":
            self.set_speed(self.left_motor,   1.0)
            self.set_speed(self.right_motor, -1.0)
            self.motor_driver.enable()
        elif msg.data == "forward":
            self.set_speed(self.left_motor,   1.0)
            self.set_speed(self.right_motor,  1.0)
            self.motor_driver.enable()
        elif msg.data == "backward":
            self.set_speed(self.left_motor,  -1.0)
            self.set_speed(self.right_motor, -1.0)
            self.motor_driver.enable()
        elif msg.data == "stop":
            self.all_stop()
        else:
            self.get_logger().error(f"Invalid cmd_str={msg.data}")

    def twist_callback(self, msg):
        speed = msg.linear.x
        heading = msg.angular.z
        # verify heading/speed are acceptable values
        # 
        if heading > 90.0 or heading < -90.0 or speed < -1.0 or speed > 1.0:
            self.get_logger().error(f"Illegal motor Twist message: speed: {speed}, heading: {heading}")
            self.all_stop()
        elif heading >= 0:
            self.set_speed(self.left_motor, cos(radians(heading) * 2) * speed)
            self.set_speed(self.right_motor, speed)
            self.motor_driver.enable()
        else:
            self.set_speed(self.left_motor, speed)
            self.set_speed(self.right_motor, cos(radians(heading) * 2) * speed)
            self.motor_driver.enable()




def main(args=None):
    rclpy.init(args=args)
    motor_node = MotorNode()

    rclpy.spin(motor_node)
    
    motor_node.all_stop()
    motor_node.destroy_node()
    rclpy.shutdown()


