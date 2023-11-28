#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from axis_camera_interfaces.msg import Axis


class Teleop(Node):

    def __init__(self):
        super().__init__('axis_teleop')

        self.use_enable_button = self.declare_parameter('use_enable_button', True).get_parameter_value().bool_value
        self.enable_button = self.declare_parameter('enable_button', 1).get_parameter_value().integer_value
        self.zoomout_button = self.declare_parameter('zoomout_button', 2).get_parameter_value().integer_value
        self.zoomin_button = self.declare_parameter('zoomin_button', 3).get_parameter_value().integer_value
        self.home_button = self.declare_parameter('home_button', 10).get_parameter_value().integer_value
        
        self.axis_pan = self.declare_parameter('axis_pan', 3).get_parameter_value().integer_value
        self.axis_tilt = self.declare_parameter('axis_tilt', 4).get_parameter_value().integer_value
        
      
        self.default_pan=2.68
        self.default_tilt=0.0
        self.default_zoom=1.0
        self.state = Axis(pan=self.default_pan, tilt=self.default_tilt, zoom=self.default_zoom)
        self.joy = None

        self.pub = self.create_publisher(Axis, "cmd", 1)
        self.sub = self.create_subscription(Joy, "/j100_0803/joy_teleop/joy", self.joy_callback, 1)

        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        self.state.brightness = 5000

        if self.joy != None and \
            ((not self.use_enable_button) or (self.joy.buttons[self.enable_button] == 1)):

            if self.joy.buttons[self.home_button] == 1:
                self.state.pan = self.default_pan
                self.state.tilt = self.default_tilt
                self.state.zoom = self.default_zoom
            else:
                self.state.pan += self.joy.axes[self.axis_pan]*5.0
                self.state.tilt += self.joy.axes[self.axis_tilt]*5.0
                self.state.zoom += self.joy.buttons[self.zoomin_button]*20.0
                self.state.zoom -= self.joy.buttons[self.zoomout_button]*20.0

            if self.state.tilt > 85.0: self.state.tilt = 85.0
            if self.state.tilt < 0.0: self.state.tilt = 0.0
            self.pub.publish(self.state)
            self.joy = None

    def joy_callback(self, data):
        self.joy = data


def main():
    
    rclpy.init()

    node = Teleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
