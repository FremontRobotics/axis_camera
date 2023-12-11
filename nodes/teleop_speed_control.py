#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from axis_camera_interfaces.msg import Axis
from std_msgs.msg import Bool

class Teleop(Node):
    def __init__(self):
        super().__init__('axis_teleop_speed_control')
        self.get_logger().info(f"Starting node")

        self.initialiseVariables()
        
        self.pub = self.create_publisher(Axis, "cmd", 1)
        self.sub = self.create_subscription(Joy, "/j100_0803/joy_teleop/joy", self.joy_callback, 1)

        timer_period = 0.2 # 5hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

#        self.pub_mirror = self.create_publisher(Bool, "mirror", 1)
        
    def initialiseVariables(self):
        self.joy = None
        self.msg = Axis() # instantiate Axis message
        self.msg.autofocus = True # autofocus is on by default
        # sensitivities[0..5] corresponding to fwd, left, up, tilt right, 
        # tilt forwards, anticlockwise twist
        self.mirror = False
        self.mirror_already_actioned = False # to stop mirror flip-flopping
        self.sensitivities = [120, -60, 40, 0, 0, 30]
        self.deadband = [0.2, 0.2, 0.2, 0.2, 0.4, 0.4]
       
    def timer_callback(self):
        if False:  #self.joy != None:
            self.createCmdMessage()
            #self.createMirrorMessage()


    def createCmdMessage(self):
        '''Creates and publishes message to command the camera.  Spacenav axes
        are: [fwd, left, up, tilt_right, tilt_forward, twist_anticlockwise'''
        self.applyThresholds()
        self.msg.pan = self.axes_thresholded[1] * self.sensitivities[1]
        self.msg.tilt = self.axes_thresholded[2] * self.sensitivities[2]
        self.msg.zoom = self.axes_thresholded[0] * self.sensitivities[0]
        #if self.joy.buttons[0]==1:
        #    self.msg.autofocus = True
        #else:
        self.msg.focus = int(self.axes_thresholded[5] * self.sensitivities[5])
        if (self.msg.focus > 0):
                # Only turn autofocus off if msg.focus!=0
                self.msg.autofocus = False
        self.pub.publish(self.msg)

    def applyThresholds(self):
        '''apply deadband to joystick output'''
        n = len(self.joy.axes)
        self.axes_thresholded = n * [0.0]
        for i in range(n):
            if (abs(self.joy.axes[i])>self.deadband[i]):
                self.axes_thresholded[i] = self.joy.axes[i]
        print(f"axes: {self.axes_thresholded}")
        
    def joy_callback(self, data):
        self.get_logger().info(f"got joy! {data}")
        self.joy = data

    def createMirrorMessage(self):
        '''Creates and publishes message to indicate image should be mirrored'''
        if self.joy.buttons[2]==1:
            if not self.mirror_already_actioned:
                self.mirror = not self.mirror
                self.mirror_already_actioned = True
        else:
            self.mirror_already_actioned = False
        self.pub_mirror.publish(self.mirror)


def main():
    rclpy.init()

    node = Teleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
