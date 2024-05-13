#!/usr/bin/env python3


import math

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

from axis_camera_interfaces.msg import Axis

#todo don't hardcode this
state_topic='/j100_0803/axis_ptz/state'

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class PublishAxisTF(Node):
    def __init__(self):
        super().__init__('publish_axis_tf')
       
        self.tf_broadcaster = TransformBroadcaster(self)

        self.subscription = self.create_subscription(
            Axis,
            state_topic,
	    self.axis_cb,
            1)
        
    def axis_cb(self, data):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = self.get_namespace() + "/pan"


        pan = math.pi + data.pan * math.pi / 180.
        tilt = -data.tilt * math.pi / 180.

        q = quaternion_from_euler(0,0,pan)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]



        self.tf_broadcaster.sendTransform(t)

        t.header.frame_id = self.get_namespace() + "/pan"
        t.child_frame_id = self.get_namespace() + "/tilt"

        q = quaternion_from_euler(0,tilt,0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = PublishAxisTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
