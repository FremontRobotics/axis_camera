#!/usr/bin/env python3
#
# Axis camera image driver. Based on:
# https://code.ros.org/svn/wg-ros-pkg/branches/trunk_cturtle/sandbox/axis_camera
# /axis.py
#

import threading
import urllib.request, urllib.error, urllib.parse
import requests, requests.auth
import datetime
import math
import time
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, CameraInfo
from std_msgs.msg import Bool
#from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
#import camera_info_manager

class StreamThread(threading.Thread):
    def __init__(self, axis):
        threading.Thread.__init__(self)
        self.axis = axis
        self.daemon = True
        self.timeoutSeconds = 2.5

    def run(self):
        while(True):
            self.stream()

    def stream(self):
        while(True):
            self.formURL()
            self.axis.get_logger().warn("formed url")
            self.authenticate()
            self.axis.get_logger().warn("authenticated")
            if self.openURL():
                self.axis.get_logger().warn("start publishing frames")
                self.publishFramesContinuously()
            time.sleep(2) # if stream stays intact we shouldn't get to this

    def formURL(self):
        self.url = 'http://%s/mjpg/video.mjpg' % self.axis.hostname
        #self.url += "?fps=%d&resolution=%dx%d" % (self.axis.fps, self.axis.width, self.axis.height)


        # support for Axis F34 multicamera switch
        if (self.axis.camera != 0):
            self.url += "&camera=%s" % str(self.axis.camera)

        print('opening ' + str(self.url))

    def authenticate(self):
        '''only try to authenticate if user/pass configured.  I have not
        used this method (yet).'''
        if self.axis.password != '' and self.axis.username != '':
            # create a password manager
            password_mgr = urllib.request.HTTPPasswordMgrWithDefaultRealm()

            # Add the username and password, use default realm.
            top_level_url = "http://" + self.axis.hostname
            password_mgr.add_password(None, top_level_url, self.axis.username,
                                                            self.axis.password)
            if self.axis.use_encrypted_password:
                handler = urllib.request.HTTPDigestAuthHandler(password_mgr)
            else:
                handler = urllib.request.HTTPBasicAuthHandler(password_mgr)

            # create "opener" (OpenerDirector instance)
            opener = urllib.request.build_opener(handler)

            # ...and install it globally so it can be used with urlopen.
            urllib.request.install_opener(opener)

    def openURL(self):
        '''Open connection to Axis camera using http'''
        try:
            self.axis.get_logger().warn(f'Opening URL: {self.url}')
            self.fp = urllib.request.urlopen(self.url, timeout=self.timeoutSeconds)
            return(True)
        except urllib.error.URLError as e:
            self.axis.get_logger().error('Error opening URL %s' % (self.url) +
                            'Possible timeout.  Looping until camera appears')
            return(False)

    def publishFramesContinuously(self):
        '''Continuous loop to publish images'''
        while(True):
            try:
                self.findBoundary()
                self.getImage()
                self.publishMsg()
                self.publishCameraInfoMsg()

            except Exception as err:
                print(f"Exception publishing frames: {err}")
                break

    def findBoundary(self):
        '''The string "--myboundary" is used to denote the start of an image in
        Axis cameras'''
        while(True):
            boundary = self.fp.readline()
            if boundary == b'--myboundary\r\n':
                break

    def getImage(self):
        '''Get the image header and image itself'''
        self.getHeader()
        self.getImageData()

    def getHeader(self):
        self.header = {}
        while(True):
            line = self.fp.readline()
            if line == b'\r\n':
                break
            line = line.strip()
            parts = line.split(b": ", 1)
            try:
                self.header[parts[0]] = parts[1]
            except:
                print('Problem encountered with image header.  Setting '
                                                    'content_length to zero')
                self.header[b'Content-Length'] = 0 # set content_length to zero if
                                            # there is a problem reading header
        self.content_length = int(self.header[b'Content-Length'])

    def getImageData(self):
        '''Get the binary image data itself (ie. without header)'''
        if self.content_length>0:
            self.img = self.fp.read(self.content_length)
            self.fp.readline() # Read terminating \r\n and do nothing with it

    def publishMsg(self):
        '''Publish jpeg image as a ROS message'''
        self.msg = CompressedImage()
        self.msg.header.stamp = self.axis.get_clock().now().to_msg()
        self.msg.header.frame_id = self.axis.frame_id   
        self.msg.format = "jpeg"
        self.msg.data = self.img
        self.axis.pub.publish(self.msg)

    def publishCameraInfoMsg(self):
        '''Publish camera info manager message'''
        camera_info = CameraInfo()
        camera_info.distortion_model = 'plumb_bob'
        camera_info.header.stamp = self.msg.header.stamp
        camera_info.header.frame_id = self.axis.frame_id
        camera_info.width = self.axis.width
        camera_info.height = self.axis.height

        cx = camera_info.width / 2.0
        cy = camera_info.height / 2.0
        fx = camera_info.width / (
            2.0 * math.tan(float(61) * math.pi / 360.0))   # TODO: Hardcoded FOV
        fy = fx
        camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.axis.caminfo_pub.publish(camera_info)

class Axis(Node):
    def __init__(self):
        
        super().__init__('axis')
    
        self.get_logger().info(f"Starting node")
        self.declare_parameter('hostname', '192.168.0.228:8001') # default IP address
        self.declare_parameter('username', '')         # default login name
        self.declare_parameter('password', '')

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 0)                   # frames per second (0 = camera default)
        self.declare_parameter('frame_id', 'axis_camera')
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('use_encrypted_password', False)
        self.declare_parameter('camera', 0)
        self.declare_parameter('ir', False)
        self.declare_parameter('defog', False)
        self.declare_parameter('wiper', False)
    
        
        self.hostname = self.get_parameter( 'hostname').get_parameter_value().string_value
        self.username = self.get_parameter('username').get_parameter_value().string_value
        self.password = self.get_parameter('password').get_parameter_value().string_value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.frame_id = self.get_parameter('frame_id').value
        self.camera_info_url = self.get_parameter('camera_info_url').value
        self.use_encrypted_password = self.get_parameter('use_encrypted_password').value
        self.camera = self.get_parameter('camera').value
        self.ir = self.get_parameter('ir').value
        self.defog = self.get_parameter('defog').value
        self.wiper = self.get_parameter('wiper').value

        self.http_headers = {
            'User-Agent': 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/111.0.0.0 Safari/537.36',
            'From': f'http://{self.hostname}'
        }
        if self.use_encrypted_password:
            self.http_auth = requests.auth.HTTPDigestAuth(self.username, self.password)
        else:
            self.http_auth = requests.auth.HTTPBasicAuth(self.username, self.password)
        self.http_timeout = (3, 5)

        # generate a valid camera name based on the hostname
        self.cname = f"AXIS_{self.hostname}"  #TODO: Improve this 
        
        self.get_logger().info(f"Camera name: {self.cname}")

        #self.cinfo.loadCameraInfo()         # required before getCameraInfo()
        self.st = None

        self.pub = self.create_publisher(CompressedImage, f"image_raw/compressed", 10)
        self.caminfo_pub = self.create_publisher(CameraInfo, f"camera_info", 10)

        if self.st is None:
            self.st = StreamThread(self)
            self.get_logger().warn(f"Starting streamthread image-publisher.")
            self.st.start()

        # The Axis Q62 series supports a night-vision mode with an active IR illuminator
        # If this option is enabled, add the necessary services and topics
        #if self.ir:
            #self.ir_on = False
            #self.ir_on_off_srv = rospy.Service('set_ir_on', SetBool, self.handle_toggle_ir)
            #self.ir_on_pub = rospy.Publisher('ir_on', Bool, queue_size=1)
            #self.ir_on_pub_thread = threading.Thread(target=self.ir_on_pub_thread_fn)
            #self.ir_on_pub_thread.start()
#
            #self.handle_toggle_ir(SetBoolRequest(False))

        # The Axis Q62 series is equipped with a wiper on the camera lens
        # If this option is enabled, add the necessary services and topics
        #if self.wiper:
            #self.wiper_on_time = datetime.datetime.utcnow()
            #self.wiper_on = False
            #self.wiper_on_off_srv = rospy.Service('set_wiper_on', SetBool, self.handle_toggle_wiper)
            #self.wiper_on_pub = rospy.Publisher('wiper_on', Bool, queue_size=1)
            #self.wiper_on_pub_thread = threading.Thread(target=self.wiper_on_pub_thread_fn)
            #self.wiper_on_pub_thread.start()
#
            #self.handle_toggle_wiper(SetBoolRequest(False))

        # The Axis Q62 series is equipped with a defogger
        # If this option is enabled, add the necessary services and topics
        #if self.defog:
            #self.defog_on = False
            #self.defog_on_off_srv = rospy.Service('set_defog_on', SetBool, self.handle_toggle_defog)
            #self.defog_on_pub = rospy.Publisher('defog_on', Bool, queue_size=1)
            #self.defog_on_pub_thread = threading.Thread(target=self.defog_on_pub_thread_fn)
            #self.defog_on_pub_thread.start()
#
            #self.handle_toggle_defog(SetBoolRequest(False))

    def __str__(self):
        """Return string representation."""
        return(self.hostname + ',' + self.username + ',' + self.password +
                       '(' + str(self.width) + 'x' + str(self.height) + ')')

    

    def handle_toggle_ir(self, req):
        """Turn the IR mode on/off (if supported)"""
        resp = SetBoolResponse()
        resp.success = True
        on_off = {
            True: "on",
            False: "off"
        }
        try:
            # Set the IR led on/off as needed
            if req.data:
                post_data = '{"apiVersion": "1.0", "method": "enableLight", "params": {"lightID": "led0"}}'
            else:
                post_data = '{"apiVersion": "1.0", "method": "disableLight", "params": {"lightID": "led0"}}'
            http_resp = requests.post(f"http://{self.hostname}/axis-cgi/lightcontrol.cgi",  post_data,
                auth=self.http_auth,
                headers=self.http_headers,
                timeout=self.http_timeout)

            if http_resp.status_code != requests.status_codes.codes.ok:
                raise Exception(f"HTTP Error setting IR illuminator: {http_resp.status_code}")

            # Enable/disable the IR filter
            if req.data:
                get_url = f"http://{self.hostname}/axis-cgi/param.cgi?action=update&PTZ.Various.V1.IrCutFilter=off&timestamp={int(time.time())}"
            else:
                get_url = f"http://{self.hostname}/axis-cgi/param.cgi?action=update&PTZ.Various.V1.IrCutFilter=on&timestamp={int(time.time())}"
            http_resp = requests.get(get_url,
                auth=self.http_auth,
                headers=self.http_headers,
                timeout=self.http_timeout)

            if http_resp.status_code != requests.status_codes.codes.ok:
                raise Exception(f"HTTP Error setting IR filter: {http_resp.status_code}")

            resp.message = f"IR mode is {on_off[req.data]}"
            self.ir_on = req.data
        except Exception as err:
            self.get_logger().warn(f"Failed to set IR mode: {err}")
            ok = False
            resp.message = str(err)

        return resp

    def ir_on_pub_thread_fn(self):
        """Publish whether the IR mode is on or off at 1Hz"""
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.ir_on_pub.publish(Bool(self.ir_on))
            rate.sleep()

    def handle_toggle_wiper(self, req):
        """Turn the wiper on/off (if supported)"""
        on_off = {
            True: "on",
            False: "off"
        }

        resp = SetBoolResponse()
        resp.success = True
        try:
            if req.data:
                post_data = '{"apiVersion": "1.0", "context": "lvc_context", "method": "start", "params": {"id": 0, "duration": 10}}'
                self.wiper_on_time = datetime.datetime.utcnow()
            else:
                post_data = '{"apiVersion": "1.0", "context": "lvc_context", "method": "stop", "params": {"id": 0}}'

            http_resp = requests.post(f"http://{self.hostname}/axis-cgi/clearviewcontrol.cgi", post_data,
                auth=self.http_auth,
                headers=self.http_headers,
                timeout=self.http_timeout)

            if http_resp.status_code != requests.status_codes.codes.ok:
                raise Exception(f"HTTP Error setting wiper: {http_resp.status_code}")

            resp.message = f"Wiper is {on_off[self.wiper_on]}"
            self.wiper_on = req.data
        except Exception as err:
            self.get_logger().warn(f"Failed to set wiper mode: {err}")
            resp.success = False
            resp.message = str(err)
        return resp

    def wiper_on_pub_thread_fn(self):
        """Publish whether the wiper is running or not at 1Hz"""
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # the wiper shuts off automatically after 10s
            if (datetime.datetime.utcnow() - self.wiper_on_time).total_seconds() > 10:
                self.wiper_on = False

            self.wiper_on_pub.publish(Bool(self.wiper_on))
            rate.sleep()

    def handle_toggle_defog(self, req):
        """Turn the defogger on/off (if supported)"""
        on_off = {
            True: "on",
            False: "off"
        }

        resp = SetBoolResponse()
        resp.success = True
        try:
            if req.data:
                get_url = f"http://{self.hostname}/axis-cgi/param.cgi?action=update&ImageSource.I0.Sensor.Defog=on&timestamp={int(time.time())}"
            else:
                get_url = f"http://{self.hostname}/axis-cgi/param.cgi?action=update&ImageSource.I0.Sensor.Defog=off&timestamp={int(time.time())}"

            http_resp = requests.get(get_url,
                auth=self.http_auth,
                headers=self.http_headers,
                timeout=self.http_timeout)

            if http_resp.status_code != requests.status_codes.codes.ok:
                raise Exception(f"HTTP Error setting defogger: {http_resp.status_code}")

            resp.message = f"Defogger is {on_off[self.defog_on]}"
            self.defog_on = req.data
        except Exception as err:
            self.get_logger().warn(f"Failed to set defogger mode: {err}")
            resp.success = False
            resp.message = str(err)
        return resp

    def defog_on_pub_thread_fn(self):
        """Publish the state of the defogger at 1Hz"""
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.defog_on_pub.publish(Bool(self.defog_on))
            rate.sleep()

def main():
    rclpy.init(args=sys.argv)

    axis = Axis()

    try:
        rclpy.spin(axis)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
