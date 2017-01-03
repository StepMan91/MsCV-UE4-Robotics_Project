#!/usr/bin/env python

""" farm_aid.py - Version 1.5.7 2015-12-20


    Created for the Robotics Project module in the master of computer vision
    in Universite De Bourgogne.

      
"""

import rospy
import actionlib
import smach
from smach import State, StateMachine, Concurrence, Container, UserData
from smach_ros import MonitorState, ServiceState, SimpleActionState, IntrospectionServer
from geometry_msgs.msg import Twist, Point, Quaternion
from std_msgs.msg import String

import cv2
import cv2.cv as cv
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

import tf
from rbx1_nav.transform_utils import quat_to_angle, normalize_angle
from math import radians, copysign, sqrt, pow, pi

from rbx2_tasks.task_setup_Grape import *

from ar_track_alvar.msg import AlvarMarkers

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
import shlex
import psutil

import subprocess
import os
import signal
import random
import numpy as np

        
class Rotate(State):
    def __init__(self):
        State.__init__(self, outcomes=['full_rotate','not_full_rotate'])

        # Initialize the tf listener
        self.tf_listener = tf.TransformListener()
        
        # Give tf some time to fill its buffer
        rospy.sleep(2)
        
        # Set the odom frame
        self.odom_frame = '/odom'
        
        # Find out if the robot uses /base_link or /base_footprint
        try:
            self.tf_listener.waitForTransform(self.odom_frame, '/base_footprint', rospy.Time(), rospy.Duration(1.0))
            self.base_frame = '/base_footprint'
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            try:
                self.tf_listener.waitForTransform(self.odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
                self.base_frame = '/base_link'
            except (tf.Exception, tf.ConnectivityException, tf.LookupException):
                rospy.loginfo("Cannot find transform between /odom and /base_link or /base_footprint")
                rospy.signal_shutdown("tf Exception")  

        self.cmd_vel = rospy.Publisher('cmd_vel', Twist)

        self.r = rospy.Rate(20)
        self.angular_speed = 0.6
        self.angular_tolerance = radians(2.5)
        self.goal_angle = 2*pi

    def execute(self, userdata):
        rospy.loginfo("Executing Rotate Class...")

        # Get the starting position values     
        (position, rotation) = self.get_odom()
        last_angle = rotation
        turn_angle = 0

        rotate_command = Twist()
        rotate_command.angular.z = self.angular_speed
        #and not userdata.detectTag_Flag
        while abs(turn_angle + self.angular_tolerance) < abs(self.goal_angle) and not rospy.is_shutdown() :

            if self.preempt_requested():
                rospy.loginfo("State Rotate is being preempted!!!")
                self.service_preempt()
               
                return 'not_full_rotate'

            # Publish the Twist message and sleep 1 cycle         
            self.cmd_vel.publish(rotate_command)
            self.r.sleep()
                
            # Get the current rotation
            (position, rotation) = self.get_odom()
                
            # Compute the amount of rotation since the last loop
            delta_angle = normalize_angle(rotation - last_angle)
                
            # Add to the running total
            turn_angle += delta_angle
            last_angle = rotation
                
        # Stop the robot before the next leg
        rotate_command = Twist()
        self.cmd_vel.publish(rotate_command)
        rospy.sleep(1)

        return 'full_rotate'

    def get_odom(self):
        # Get the current transform between the odom and base frames
        try:
            (trans, rot)  = self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        except (tf.Exception, tf.ConnectivityException, tf.LookupException):
            rospy.loginfo("TF Exception")
            return

        return (Point(*trans), quat_to_angle(Quaternion(*rot)))

class DetectTag(smach.State):
    def __init__(self):
        rospy.loginfo('Initilizing DetectTag')
        smach.State.__init__(self, outcomes=['detect0','detect1','detect2','detect3','detect_other', 'not_find'])
        rospy.Subscriber("ar_pose_marker", AlvarMarkers, self.tagDetected)
        self.tag_ids = [6,5,3,2]

        self.my_tag = 999
        self.number_of_marker = 0
        self.flag=False
        #self.positionX = 0

    def execute(self, userdata):
        rospy.loginfo('Executing DetectTag Class...')
        self.flag = True
        
        #and self.positionX < d
        while(self.number_of_marker==0 ):

            if(self.preempt_requested()):
                rospy.loginfo("state DetectTag is being preempted!!")
                self.service_preempt()

                self.my_tag = 999
                self.number_of_marker=0
                self.flag = False
                return 'not_find'

            rospy.sleep(1)

     
        # and self.positionX < d
        if(self.my_tag==2 ):
            rospy.loginfo('detected 2' )
            self.my_tag = 999
            self.flag = False
            self.number_of_marker=0
            return 'detect0'

        elif (self.my_tag==3 ):
            rospy.loginfo('detected 3')
            self.my_tag = 999
            self.number_of_marker=0
            self.flag = False
            return 'detect1'

        elif (self.my_tag==5 ):
            rospy.loginfo('detected 5')
            self.my_tag = 999
            self.number_of_marker=0
            self.flag = False
            return 'detect2'

        elif (self.my_tag==6 ):
            rospy.loginfo('detected 7')
            self.my_tag = 999
            self.number_of_marker=0
            self.flag = False
            return 'detect3'

        else :
            rospy.loginfo( 'other tag than the pre defined')
            self.number_of_marker=0
            self.flag = False
            return 'detect_other'
            

    def tagDetected(self, msg):
        #rospy.loginfo('trying to detect')
        if self.flag:

            # Get the number of markers
            self.number_of_marker = len(msg.markers)
            
            # If no markers detected, just return
            if self.number_of_marker == 0:
                return

            # Iterate through the tags and sum the x, y and z coordinates            
            for tag in msg.markers:
                
                # Skip any tags that are not in our list
                if self.tag_ids is not None and not tag.id in self.tag_ids:
                    continue

                rospy.loginfo('detect something')
                self.my_tag = tag.id
                self.positionX=tag.pose.pose.position.x
                #rospy.loginfo('detected tag: %d', tag.id,'and distance is %d',tag.pose.pose.position.x)

class top_state(State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])
    def execute(self):
        rospy.loginfo('Executing state Start Robot')
        rospy.sleep(1)
        return 'start'

class Patrol():
    def __init__(self):
        rospy.init_node('patrol_smach', anonymous=False)
                
        # Initialize a number of parameters and variables
        setup_task_environment(self)
        
        # Track success rate of getting to the goal locations
        self.n_succeeded = 0
        self.n_aborted = 0
        self.n_preempted = 0
        self.bridge = CvBridge()

        # A list to hold then navigation waypoints
        nav_states = list()
        
        # Turn the waypoints into SMACH states
        for waypoint in self.waypoints:           
            nav_goal = MoveBaseGoal()
            nav_goal.target_pose.header.frame_id = 'map'
            nav_goal.target_pose.pose = waypoint
            move_base_state = SimpleActionState('move_base', MoveBaseAction, goal=nav_goal, result_cb=self.move_base_result_cb,
                                                 exec_timeout=rospy.Duration(90.0),
                                                 server_wait_timeout=rospy.Duration(10.0))
            nav_states.append(move_base_state)


        home_goal=MoveBaseGoal()
        home_goal.target_pose.header.frame_id='map'
        home2=Pose(Point(0.0,0.0,0.0),Quaternion(0,0,0,1))
        home_goal.target_pose.pose=home2
        move_base_state_home=SimpleActionState('move_base', MoveBaseAction, goal=home_goal, result_cb=self.move_base_result_cb,
                                                 exec_timeout=rospy.Duration(120.0),
                                                 server_wait_timeout=rospy.Duration(10.0))
        home_state=move_base_state_home

        # Create the nav_patrol state machine using a Concurrence container
        self.detect_rotate=Concurrence(outcomes=['detect0','detect1','detect2','detect3','detect_other', 'not_find'],
                                        default_outcome='not_find',
                                        child_termination_cb=self.concurrence_child_termination_cb,
                                        outcome_cb=self.concurrence_outcome_cb)
        
        with self.detect_rotate:
            Concurrence.add('DETECT_TAG', DetectTag())
            Concurrence.add('ROTATING', Rotate())

        # Initialize the patrol state machine
        self.sm_patrol = StateMachine(outcomes=['succeeded','aborted','preempted'])

        # Add the states to the state machine with the appropriate transitions
        with self.sm_patrol:
            StateMachine.add('GO_TO_0', nav_states[0], transitions={'succeeded':'GO_TO_detect_rotate', 'aborted':'GO_TO_0'} )
            StateMachine.add('GO_TO_1', nav_states[1], transitions={'succeeded':'GO_TO_detect_rotate', 'aborted':'GO_TO_0'})
            StateMachine.add('GO_TO_2', nav_states[2], transitions={'succeeded':'GO_TO_detect_rotate', 'aborted':'GO_TO_0'} )
            StateMachine.add('GO_TO_3', nav_states[3], transitions={'succeeded':'GO_TO_detect_rotate', 'aborted':'GO_TO_0'} )
            
            StateMachine.add('GO_TO_detect_rotate', self.detect_rotate,transitions={'detect0':'GO_TO_1','detect1':'GO_TO_2','detect2':'GO_TO_3','detect3':'GO_TO_0', 'detect_other':'GO_TO_0', 'not_find':'GO_TO_detect_rotate'})
        
        # Create and start the SMACH introspection server

        intro_server = IntrospectionServer('patrol_in', self.sm_patrol, '/SM_ROOT')
        intro_server.start()

        # Set the shutdown function (stop the robot)
        sm_outcome = self.sm_patrol.execute()
        rospy.loginfo('State Machine Outcome: ' + str(sm_outcome))
        
        #rospy.on_shutdown(self.shutdown)
        intro_server.stop()

    def concurrence_child_termination_cb(self, outcome_map):
        if outcome_map['DETECT_TAG']=='detect0' or outcome_map['DETECT_TAG']=='detect1' or outcome_map['DETECT_TAG']=='detect2' or outcome_map['DETECT_TAG']=='detect3' or outcome_map['DETECT_TAG']=='detect_other':
            rospy.loginfo('preempted due to detection of tag')
            return True
        if outcome_map['ROTATING']=='full_rotate':
            rospy.loginfo('preempted in the due to full rotation')
            return True

        return False

    def concurrence_outcome_cb(self, outcome_map):
        if outcome_map['DETECT_TAG']=='detect0':
            rospy.loginfo('preemption tag 0 in concurrence_outcome_cb')
            return 'detect0'

        elif outcome_map['DETECT_TAG']=='detect1':
            rospy.loginfo('preemption tag 0 in concurrence_outcome_cb')
            return 'detect1'

        elif outcome_map['DETECT_TAG']=='detect2':
            rospy.loginfo('preemption tag 0 in concurrence_outcome_cb')
            return 'detect2'

        elif outcome_map['DETECT_TAG']=='detect3':
            rospy.loginfo('preemption tag 0 in concurrence_outcome_cb')
            return 'detect3'

        elif outcome_map['DETECT_TAG']=='detect_other':
            rospy.loginfo('preemption tag 0 in concurrence_outcome_cb')
            return 'detect_other'

        elif outcome_map['ROTATING']=='full_rotate':
            rospy.loginfo('it is full_rotate in the concurrancy outcome')
            return 'not_find'


        else:
            return 'not_find'
            
        
    def image_callback(self, userdata, ros_image):
        
        
        # Use cv_bridge() to convert the ROS image to OpenCV format
        #rospy.loginfo('initiating...')
        try:
            frame = self.bridge.imgmsg_to_cv(ros_image, "bgr8")
            
        except CvBridgeError, e:
            print e
            
        
        #rospy.loginfo('IMAGE RECEIVED')
        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        frame = np.array(frame, dtype=np.uint8)
        
        # Process the frame using the process_image() function
        display_image = self.process_image(frame)
                     
        # Display the image.
        cv2.imshow("Image_Window", display_image)
        
        # Process any keyboard commands
        self.keystroke = cv.WaitKey(5)
       
        #rospy.loginfo('conc 1')
        rospy.sleep(10)
        return True
    
    
    def process_image(self, frame):
        # Convert to greyscale
        grey = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
        
        # Blur the image
       # grey = cv2.blur(grey, (7, 7))
        
        # Compute edges using the Canny edge filter
        edges = cv2.Canny(grey, 15.0, 30.0)
        
        return edges
    
       
    def image_callback1(self, userdata, ros_image):
        
        
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            frame = self.bridge.imgmsg_to_cv(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
            rospy.loginfo('ERROR ERROR!!!')
        #rospy.loginfo('IMAGE RECEIVED')
        # Convert the image to a Numpy array since most cv2 functions
        # require Numpy arrays.
        frame = np.array(frame, dtype=np.uint8)
        
        # Process the frame using the process_image() function
        display_image = self.process_image1(frame)
        #rospy.loginfo('image view window final')
               
        # Display the image.
        cv2.imshow("Image_Window1", display_image)
        
        # Process any keyboard commands
        self.keystroke = cv.WaitKey(5)

        

    def process_image1(self, frame):
        # Convert to greyscale
        grey = cv2.cvtColor(frame, cv.CV_BGR2GRAY)
        
        # Blur the image
        grey = cv2.blur(grey, (7, 7))
        
        # Compute edges using the Canny edge filter
        #edges = cv2.Canny(grey, 15.0, 30.0)
        
        return grey
    

    def move_base_result_cb(self, userdata, status, result):
        if status == actionlib.GoalStatus.SUCCEEDED:
            self.n_succeeded += 1
        elif status == actionlib.GoalStatus.ABORTED:
            self.n_aborted += 1
        elif status == actionlib.GoalStatus.PREEMPTED:
            self.n_preempted += 1

        try:
            #rospy.loginfo("n_succeeded : " + str(self.n_succeeded))
            #rospy.loginfo("n_aborted : " + str(self.n_aborted))
            #rospy.loginfo("n_preempted : " + str(self.n_preempted))
            rospy.loginfo("Success rate: " + str(100.0 * self.n_succeeded / (self.n_succeeded + self.n_aborted  + self.n_preempted)))
        except:
            pass

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        
        self.sm_patrol.request_preempt()        
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        Patrol()
    except rospy.ROSInterruptException:
        rospy.loginfo("SMACH test finished.")
