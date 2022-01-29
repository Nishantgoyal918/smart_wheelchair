#!/usr/bin/env python3

from typing import Union
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header

import numpy as np
import math

from ai2thor.controller import Controller
from ai2thor.server import Event



'''
Config:
'''
config = {

    # ROS Params
    'ROS_NODE_NAME': "ai2thor_interface",

    # PUBLISH
    'RGB_TOPIC': "/camera/rgb/image_raw",
    'DEPTH_TOPIC': "/camera/depth/image_raw",
    'CAMERA_RGB_INFO_TOPIC': "/camera/rgb/camera_info",
    'CAMERA_DEPTH_INFO_TOPIC': "/camera/depth/camera_info",
    
    'PUB_QUEUE_SIZE': 20,
    'PUB_RATE': 20,

    # SUB
    'VEL_TOPIC': "/cmd_vel",

    # MOVE_BASE
    'CONTROLLER_FREQ': 10,



    # Ai2Thor Params
    'AGENT_MODE': "locobot",
    'VISIBILITY_DIST': 100, #1.5,

    'SCENE': "FloorPlan_Train1_3",
    
    'GRID_SIZE': 0.001,
    'MOVEMENT_GAUSS_SIGMA': 0.005,

    'CAM_WIDTH': 640,
    'CAM_HEIGHT': 480,
    'CAM_FOV': 90, #70,

}




'''
Ai2ThorNode Class
'''
class Ai2ThorNode:


    def __init__(
        self,
        config: dict,
        ) -> None:

        '''
        Ai2ThorNode class

        Starts the ROS interface

        Params:

            node_name (str)         : Name of the ROS Node
        '''

        self.config = config

        self.controller_freq = self.config['CONTROLLER_FREQ']

        # Start ROS Node
        rospy.init_node(self.config['ROS_NODE_NAME'])
        self.rate = rospy.Rate(self.config['PUB_RATE'])


        # Ai2Thor Controller
        self.controller = Controller(

            agentMode=self.config['AGENT_MODE'],
            visibilityDistance=self.config['VISIBILITY_DIST'],
            scene=self.config['SCENE'],
            gridSize=self.config['GRID_SIZE'],
            movementGaussianSigma=self.config['MOVEMENT_GAUSS_SIGMA'],
            
            width=self.config['CAM_WIDTH'],
            height=self.config['CAM_HEIGHT'],
            fieldOfView=self.config['CAM_FOV'],
            renderDepthImage=True,
        )


        self.action_msg = None
        self.update_action_msg = None
        self.action_update_time = None


        # CVBridge
        self.cv_bridge = CvBridge()

        # ROS Publishers
        self.rgb_publisher = rospy.Publisher(self.config['RGB_TOPIC'], Image, queue_size=self.config['PUB_QUEUE_SIZE'])
        self.depth_publisher = rospy.Publisher(self.config['DEPTH_TOPIC'], Image, queue_size=self.config['PUB_QUEUE_SIZE'])
        self.camera_rgb_info_publisher = rospy.Publisher(self.config['CAMERA_RGB_INFO_TOPIC'], CameraInfo, queue_size=self.config['PUB_QUEUE_SIZE'])
        self.camera_depth_info_publisher = rospy.Publisher(self.config['CAMERA_DEPTH_INFO_TOPIC'], CameraInfo, queue_size=self.config['PUB_QUEUE_SIZE'])

        # ROS Subscribers
        self.vel_subscriber = rospy.Subscriber(self.config['VEL_TOPIC'], Twist, callback=self.velocity_callback)



    def update_action(
        self,
        ) -> None:
        if self.action_update_time is None:
            self.update_action_msg = None
        else:
            if self.action_update_time - rospy.Time.now().to_sec() >= 10 * (1 / self.controller_freq):
                self.update_action_msg = None


    def simulate(
        self,
        ) -> None:

        '''
        simulate: Simulation Loop
        '''
        
        while True:

            self.step(self.action_msg)
            self.update_action()
            self.action_msg = self.update_action_msg
            # self.action_msg = None # Commented This
            self.rate.sleep() # Commented This





    def velocity_callback(
        self,
        velocity_msg: Twist) -> None:
        '''
        velocity_callback: Callback for MOVE_BASE velocity subscriber
                           Converts velocity_msg to Ai2Thor action

        Params:
            velocity_msg (geometry_msgs.Twist) : Velocity Msg received by vel_subscriber
        '''

        forward_dist = velocity_msg.linear.x * (1 / self.controller_freq)
        angular_dist = velocity_msg.angular.z * (1 / self.controller_freq)

        # print("VELOCITY: ", forward_dist, ", ", angular_dist)

        self.action_msg = {
            'FORWARD': forward_dist,
            'ROTATE': (angular_dist * 180) / math.pi
        }

        self.update_action_msg = self.action_msg
        self.action_update_time = rospy.Time.now().to_sec()
        # self.step(action_msg)

    

    def step(
        self,
        action_msg: Union[dict, None] = None
        ) -> None:

        '''
        step: Performs action on the Ai2Thor Agent

        Params:
            action_msg (Union[dict, None]) : Action that need to be performed on the agent. Does nothing if None is passed
        '''

        if action_msg is None:
            
            self.publish_state(    
                    self.controller.step(
                        action = "MoveAhead",
                        moveMagnitude = 0.0
                    )
                )

        else:
            # Move Forward
            self.publish_state(
                self.controller.step(
                    action = "MoveAhead",
                    moveMagnitude = action_msg['FORWARD']
                )
            )
            
            
            # Rotate
            if action_msg['ROTATE'] >= 0.001:

                self.publish_state(
                    self.controller.step(
                        action = "RotateLeft",
                        degrees = abs(action_msg['ROTATE'])
                    )
                )
            
            elif action_msg['ROTATE'] <= -0.001:

                self.publish_state(
                    self.controller.step(
                        action = "RotateRight",
                        degrees = abs(action_msg['ROTATE'])
                    )
                )
            
            else:

                pass
            
        # self.action_msg = None # Uncommented This
        # self.rate.sleep() # Uncommented This
                

    
    def publish_state(
        self,
        event: Event
        ) -> None:

        '''
        publish_state: Publishes the current environment state from Ai2Thor

        Params:
            event (ai2thor.server.Event)  : event that contains the state to be published
        '''
        header = self.to_header(rospy.Time.now())

        self.rgb_publisher.publish(self.convert_to_img_msg(event.frame, header, "rgb8"))
        # self.depth_publisher.publish(self.convert_to_img_msg(event.depth_frame.astype('uint16'), header, "mono16"))
        self.depth_publisher.publish(self.convert_to_img_msg(event.depth_frame, header, "mono16"))

        camera_info_msg = self.make_camera_info_msg(header, height=self.config['CAM_HEIGHT'], 
                                                                        width=self.config['CAM_WIDTH'])

        self.camera_rgb_info_publisher.publish(camera_info_msg)
        self.camera_depth_info_publisher.publish(camera_info_msg)

        pass


    def convert_to_img_msg(
        self,
        cv2img: np.array,
        header: Header,
        encoding: str,
        ) -> Image:

        '''
        convert_to_img_msg: Converts the CV2 image to image Msg

        Params:
            cv2img (np.array)   : CV2 Image (BGR or Grayscale)
            header (Header)     : Message Header

        return:
            Returns the ROS sensor_msgs.msg.Image
        '''

        img_msg = self.cv_bridge.cv2_to_imgmsg(cv2img)
        img_msg.header = header
        
        if encoding == "rgb8":
            img_msg.encoding = encoding

        return img_msg


    
    def to_header(
        self,
        timestamp: rospy.Time
        ) -> Header:
        
        '''
        to_header: Message Header

        Params:
            timestamp (rospy.Time) : Observation Time
        
        return:
            Returns the ROS message Header (std_msgs.msg.Header)
        '''

        header = Header()
        header.stamp = timestamp
        header.frame_id = "camera"

        return header



    def make_camera_info_msg(
        self,
        header: Header,
        height: float,
        width: float
        ) -> CameraInfo:

        '''
        make_camera_info_msg: Makes CameraInfo message

        Params:
            header (Header)  : Message Header
            height (float)   : Image Height
            width  (float)   : Image Width
        
        return:
            Returns the ROS sensor_msgs.msg.CameraInfo
        '''

        camera_info_msg = CameraInfo()
        camera_info_msg.header = header
        
        fx, fy = width / 2, height / 2
        cx, cy = width / 2, height / 2

        camera_info_msg.width = width
        camera_info_msg.height = height
        camera_info_msg.distortion_model = "plumb_bob"
        camera_info_msg.K = np.float32([fx, 0, cx, 0, fy, cy, 0, 0, 1])
        camera_info_msg.D = np.float32([0, 0, 0, 0, 0])
        camera_info_msg.P = [fx, 0, cx, 0, 0, fy, cy, 0, 0, 0, 1, 0]

        return camera_info_msg


node = Ai2ThorNode(config)
node.simulate()