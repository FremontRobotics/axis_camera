#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from axis_camera_interfaces.msg import Axis


class Teleop(Node):

    def __init__(self):
        super().__init__('axis_teleop')

        self.enable_button = self.declare_parameter('~enable_button', 3).get_parameter_value().integer_value
        self.axis_pan = self.declare_parameter('~axis_pan', 3).get_parameter_value().integer_value
        self.axis_tilt = self.declare_parameter('~axis_tilt', 4).get_parameter_value().integer_value
        
        self.state = Axis(pan=220.0)
        self.joy = None

        self.pub = self.create_publisher(Axis, "cmd", 1)
        print('creating sub!')
        self.sub = self.create_subscription(Joy, "j100_0803/joy_teleop/joy", self.joy_callback, 1)
        

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.state.brightness = 5000

        if self.joy != None and self.joy.buttons[self.enable_button] == 1:
            self.get_logger().warn(f'Setting {str(self.joy)}')
            #and (rospy.Time.now() - self.joy.header.stamp).to_sec() < 0.2:
            self.state.pan += self.joy.axes[self.axis_pan]*5.0
            self.state.tilt += self.joy.axes[self.axis_tilt]*5.0
            if self.state.tilt > 85.0: self.state.tilt = 85.0
            if self.state.tilt < 0.0: self.state.tilt = 0.0
            self.pub.publish(self.state)

    def joy_callback(self, data):
        self.joy = data


def main():
    
    rclpy.init()

    node = Teleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
