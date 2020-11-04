from time import sleep
import rclpy
from rclpy.node import Node

import qwiic
from sensor_msgs.msg import Range


class Vl53l1xNode(Node):

    def __init__(self):
        super().__init__('vl53l1x_node')
        
        self.sensor = qwiic.QwiicVL53L1X()

        if (not self.init_sensor()):
           self.get_logger().error("failed init_sensor() during construction, destroying node") 
           self.destroy_node()

        self.error_count = 0
        self.distance_msg = Range()
        self.distance_msg.radiation_type = Range.INFRARED
        self.distance_msg.field_of_view = 0.471     # 27 degrees
        self.distance_msg.min_range = .04           # 40mm
        self.distance_msg.max_range = 1.3           # 1.3m (short distance mode)


        self.distance_pub = self.create_publisher(Range, 'tof_distance', 10)
        timer_period = .125     # 125ms
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def init_sensor(self):
        if (status := self.sensor.sensor_init()):
           self.get_logger().error(f"sensor_init() failed, error code: {status}") 
           return False
        else:
            self.sensor.set_distance_mode(1)    # short distance mode
            self.sensor.clear_interrupt()
            self.sensor.start_ranging()
            return True
    
    def reset_sensor(self):
        self.sensor.soft_reset()
        sleep(.01)
        if (not self.init_sensor()):
            self.get_logger().error("failed init_sensor() during reset") 
            self.sensor.clear_interrupt()
        else:
            self.get_logger().info("sensor successfully reset")



    def timer_callback(self):
        if (self.sensor.check_for_data_ready()):
            self.sensor.clear_interrupt()
            self.error_count = 0
            self.distance_msg.range = float(self.sensor.get_distance())
            self.distance_msg.header.stamp = rospy.Time.now()
            self.distance_pub.publish(self.distance_msg)
#            self.get_logger().info(f"publishing distance: {self.distance_msg.range}")
        else:
            self.error_count += 1
            if (self.error_count >= 5):
                self.get_logger().error("max error count hit, resetting...")
                self.reset_sensor()

def main(args=None):
    rclpy.init(args=args)

    vl53l1x_node = Vl53l1xNode()

    rclpy.spin(vl53l1x_node)

    vl53l1x_node.destroy_node()
    rclpy.shutdown()

