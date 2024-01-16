#!/usr/bin/env python3
#
# Basic PTZ node, based on documentation here:
#   http://www.axis.com/files/manuals/vapix_ptz_45621_en_1112.pdf
#
import threading
import requests, requests.auth
import urllib.parse
import math
import sys

import rclpy
from rclpy.node import Node
from axis_camera_interfaces.msg import Axis
from std_msgs.msg import Bool


class StateThread(threading.Thread):
    '''This class handles the publication of the positional state of the camera
    to a ROS message'''

    def __init__(self, axis):
        threading.Thread.__init__(self)
        self.axis = axis
        # Permit program to exit even if threads are still running by flagging
        # thread as a daemon:
        self.daemon = True

    def run(self):
        rate = self.axis.create_rate(1)
        self.msg = Axis()

        while True:
            self.queryCameraPosition()
            self.publishCameraState()
            rate.sleep()

    def queryCameraPosition(self):
        '''Using Axis VAPIX protocol, described in the comments at the top of
        this file, is used to query the state of the camera'''

        queryParams = { 'query':'position' }

        try:
            url = f"http://{self.axis.hostname}/axis-cgi/com/ptz.cgi?{urllib.parse.urlencode(queryParams)}"
            resp = requests.get(url, auth=self.axis.http_auth, timeout=self.axis.http_timeout, headers=self.axis.http_headers)

            if resp.status_code == requests.status_codes.codes.ok:
                # returns a string of the form
                #   pan=-0.01
                #   tilt=-45.03
                #   zoom=1
                #   iris=5748
                #   focus=4642
                #   brightness=4999
                #   autofocus=on
                #   autoiris=on
                new_camera_position = {}
                body = resp.text.split()
                for row in body:
                    if '=' in row:
                        (key, value) = row.split('=')
                        new_camera_position[key.strip()] = value.strip()

                self.cameraPosition = new_camera_position
            else:
                raise Exception(f"HTTP Error querying the camera position: {resp.status_code}")

        except Exception as e:
            exception_error_str = "Exception: '" + str(e) + "' when querying the url: http://" + \
                                  self.axis.hostname + "/axis-cgi/com/ptz.cgi?%s" % urllib.parse.urlencode(queryParams)
            self.axis.get_logger().warn(exception_error_str)

            self.cameraPosition = None

    def publishCameraState(self):
        '''Publish camera state to a ROS message'''
        try:
            print(self.cameraPosition)
            if self.cameraPosition is not None:
                self.msg.pan = float(self.cameraPosition['pan'])
                if self.axis.flip:
                    self.adjustForFlippedOrientation()
                self.msg.tilt = float(self.cameraPosition['tilt'])
                self.msg.zoom = float(self.cameraPosition['zoom'])
                if 'brightness' in self.cameraPosition:
                    self.msg.brightness = int(self.cameraPosition['brightness'])
                self.msg.iris = 0.0
                if 'iris' in self.cameraPosition:
                    self.msg.iris = float(self.cameraPosition['iris'])
                self.msg.focus = 0
                if 'focus' in self.cameraPosition:
                    self.msg.focus = int(self.cameraPosition['focus'])
                if 'autofocus' in self.cameraPosition:
                    self.msg.autofocus = (self.cameraPosition['autofocus'] == 'on')
                if 'autoiris' in self.cameraPosition:
                    self.msg.autoiris = (self.cameraPosition['autoiris'] == 'on')
                self.axis.pub.publish(self.msg)
        except KeyError as e:
            self.axis.get_logger().warn("Camera not ready for polling its telemetry: " + str(e))

    def adjustForFlippedOrientation(self):
        '''Correct pan and tilt parameters if camera is mounted backwards and
        facing down'''
        self.msg.pan = 180 - self.msg.pan
        if self.msg.pan > 180:
            self.msg.pan -= 360
        elif self.msg.pan < -180:
            self.msg.pan += 360
        self.msg.tilt = -self.msg.tilt

class AxisPTZ(Node):
    '''This class creates a node to manage the PTZ functions of an Axis PTZ
    camera'''

    def __init__(self):
        super().__init__('axis_ptz')

        self.get_logger().info(f"Starting node axis_ptz")

        self.declare_parameter('hostname', '192.168.0.228:8001') # default IP address
        self.declare_parameter('username', '')         # default login name
        self.declare_parameter('password', '')
        self.declare_parameter('frame_id', 'axis_camera')
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('use_encrypted_password', False)
        self.declare_parameter('flip', False)
        self.declare_parameter('speed_control', False)
        self.declare_parameter('mirror', False)

        self.hostname = self.get_parameter('hostname').get_parameter_value().string_value
        self.flip = self.get_parameter('flip').get_parameter_value().bool_value

        self.pub = self.create_publisher(Axis, "state", 10)
        self.sub = self.create_subscription(Axis, "cmd", self.cmd, 1)
        self.sub_mirror = self.create_subscription(Bool, "mirror", self.mirrorCallback, 1)

        self.st = None

        if not self.get_parameter('username').get_parameter_value().string_value:
            self.http_auth = None
        elif self.get_parameter('use_encrypted_password').get_parameter_value().bool_value:
            self.http_auth = requests.auth.HTTPDigestAuth(
                self.get_parameter('username').get_parameter_value().string_value, 
                self.get_parameter('password').get_parameter_value().string_value)
        else:
            self.http_auth = requests.auth.HTTPBasicAuth(
                self.get_parameter('username').get_parameter_value().string_value, 
                self.get_parameter('password').get_parameter_value().string_value)
        
        self.http_headers = {
            'User-Agent': 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/111.0.0.0 Safari/537.36',
            'From': f'http://{self.hostname}'
        }
        self.http_timeout = (3, 5)


        if self.st is None:
            self.st = StateThread(self)
            self.get_logger().warn(f"Starting StateThread")
            self.st.start()

    def cmd(self, msg):
        '''Command the camera with speed control or position control commands'''
        self.msg = msg
        if self.get_parameter('flip').get_parameter_value().bool_value: 
            self.adjustForFlippedOrientation()
        if self.get_parameter('mirror').get_parameter_value().bool_value:
            self.msg.pan = -self.msg.pan
        self.sanitisePTZCommands()
        self.applySetpoints()

    def adjustForFlippedOrientation(self):
        '''If camera is mounted backwards and upside down (ie. self.flip==True
        then apply appropriate transforms to pan and tilt'''
        self.msg.tilt = -self.msg.tilt
        if self.get_parameter('speed_control').get_parameter_value().bool_value:
            self.msg.pan = -self.msg.pan
        else:
            self.msg.pan = 180.0 - self.msg.pan

    def sanitisePTZCommands(self):
        '''Applies limits to message and corrects for flipped camera if
        necessary'''
        self.sanitisePan()
        self.sanitiseTilt()
        self.sanitiseZoom()
        self.sanitiseFocus()
        self.sanitiseBrightness()
        self.sanitiseIris()

    def sanitisePan(self):
        '''Pan speed (in percent) must be: -100<pan<100'
        Pan must be: -180<pan<180 even though the Axis cameras can only achieve
        +/-170 degrees rotation.'''
        if self.get_parameter('speed_control').get_parameter_value().bool_value:
            if abs(self.msg.pan)>100.0:
                self.msg.pan = math.copysign(100.0, self.msg.pan)
        else: # position control so need to ensure -180<pan<180:
            self.msg.pan = ((self.msg.pan + 180.0) % 360.0) - 180.0

    def sanitiseTilt(self):
        '''Similar to self.sanitisePan() but for tilt'''
        if self.get_parameter('speed_control').get_parameter_value().bool_value:
            if abs(self.msg.tilt)>100.0:
                self.msg.tilt = math.copysign(100.0, self.msg.tilt)
        else: # position control so ensure tilt: -180<tilt<180:
            self.msg.tilt = ((self.msg.tilt + 180.0) % 360) - 180.0

    def sanitiseZoom(self):
        '''Zoom must be: 1<zoom<9999.  continuouszoommove must be:
        -100<zoom<100'''
        if self.get_parameter('speed_control').get_parameter_value().bool_value:
            if abs(self.msg.zoom)>100:
                self.msg.zoom = math.copysign(100.0, self.msg.zoom)
        else: # position control:
            if self.msg.zoom>9999.0:
                self.msg.zoom = 9999.0
            elif self.msg.zoom<1.0:
                self.msg.zoom = 1.0

    def sanitiseFocus(self):
        '''Focus must be: 1<focus<9999.  continuousfocusmove: -100<rfocus<100'''
        if self.get_parameter('speed_control').get_parameter_value().bool_value:
            if abs(self.msg.focus)>100:
                self.msg.focus = math.copysign(100.0, self.msg.focus)
        else: # position control:
            if self.msg.focus>9999:
                self.msg.focus = 9999
            elif self.msg.focus < 1:
                self.msg.focus = 1

    def sanitiseBrightness(self):
        '''Brightness must be: 1<brightness<9999.  continuousbrightnessmove must
        be: -100<rbrightness<100.  Note that it appears that the brightness
        cannot be adjusted on the Axis 214PTZ'''
        if self.get_parameter('speed_control').get_parameter_value().bool_value:
            if abs(self.msg.brightness) > 100:
                self.msg.brightness = int(math.copysign(100, self.msg.brightness))
        else: # position control:
            if self.msg.brightness>9999:
                self.msg.brightness = 9999
            elif self.msg.brightness<1:
                self.msg.brightness = 1

    def sanitiseIris(self):
        '''Iris value is read only because autoiris has been set to "on"'''
        if self.msg.iris>0.000001:
            self.get_logger().warn("Iris value is read-only.")

    def applySetpoints(self):
        '''Apply set-points to camera via HTTP'''

        self.createCmdString()
        #self.get_logger().info(f"Sending cmdString: {self.cmdString}")
        try:
            url = f"http://{self.hostname}/{self.cmdString}"
            resp = requests.get(url, auth=self.http_auth, timeout=self.http_timeout, headers=self.http_headers)

            if resp.ok:
                pass
            else:
                raise Exception(f"HTTP error {resp.status_code}")

        except Exception as e:
            self.get_logger().warn(f'Failed to connect to camera to send command message: {e}')

    def createCmdString(self):
        '''creates http cgi string to command PTZ camera'''
        self.cmdString = '/axis-cgi/com/ptz.cgi?'
        if self.get_parameter('speed_control').get_parameter_value().bool_value :
            self.cmdString += 'continuouspantiltmove=%d,%d&' % \
                                    (int(self.msg.pan), int(self.msg.tilt)) \
                    + 'continuouszoommove=%d&' % (int(self.msg.zoom)) \
                    + 'continuousbrightnessmove=%d&' % \
                                                    (int(self.msg.brightness))
            # Note that brightness adjustment has no effect for Axis 214PTZ.
            if True: #self.msg.autofocus:
                self.cmdString += 'autofocus=on&'
            else:
                self.cmdString += 'autofocus=off&continuousfocusmove=%d&' % \
                                                        (int(self.msg.focus))
            if self.msg.autoiris:
                self.cmdString += 'autoiris=on'
            else:
                self.cmdString += 'autoiris=off'
        else: # position control:
            self.cmdString += 'pan=%d&tilt=%d&' % (self.msg.pan, self.msg.tilt)\
                        + 'zoom=%d&' % (int(self.msg.zoom)) \
                        + 'brightness=%d&' % (int(self.msg.brightness))
            if self.msg.autofocus:
                self.cmdString += 'autofocus=on&'
            else:
                self.cmdString += 'autofocus=off&focus=%d&' % \
                                                        (int(self.msg.focus))
            if self.msg.autoiris:
                self.cmdString += 'autoiris=on'
            else:
                self.cmdString += 'autoiris=off'

    def mirrorCallback(self, msg):
        '''Command the camera with speed control or position control commands'''
        self.mirror = msg.data

    def callback(self, config, level):
        #self.speedControl = config.speed_control

        # create temporary message and fill with data from dynamic reconfigure
        temp_msg = Axis()
        temp_msg.pan = config.pan
        temp_msg.tilt = config.tilt
        temp_msg.zoom = config.zoom
        temp_msg.focus = config.focus
        temp_msg.brightness = config.brightness
        temp_msg.autofocus = config.autofocus

        # check sanity and apply values
        self.cmd(temp_msg)

        # read sanitized values and update GUI
        config.pan = self.msg.pan
        config.tilt = self.msg.tilt
        config.zoom = self.msg.zoom
        config.focus = self.msg.focus
        config.brightness = self.msg.brightness
        config.autofocus = self.msg.autofocus

        # update GUI with sanitized values
        return config

def main():
    
    rclpy.init(args=sys.argv)

    ptz = AxisPTZ()

    try:
        rclpy.spin(ptz)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
