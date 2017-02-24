#!/usr/bin/env python

'''
Copyright (c) 2016, Allgeyer Tobias, Aumann Florian, Borella Jocelyn, Hutmacher Robin, Karrenbauer Oliver, Marek Felix, Meissner Pascal, Trautmann Jeremias, Wittenbeck Valerij

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import roslib
import rospy
import smach
import smach_ros
import numpy
import time
import math
import timeit

from ptu_controller.msg import PTUMovementGoal, PTUMovementAction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import (Pose, PoseWithCovariance, PoseWithCovarianceStamped)
from asr_robot_model_services.srv import CalculateCameraPoseCorrection
from nav_msgs.msg import Odometry
from actionlib import *
from actionlib.msg import *
from evaluation_decorators import *
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import state_acquisition
import __builtin__

"""
Defines states that are relevant for moving the robot, i.e. moving the base,
moving the ptu and getting the current positition.
"""

# Make independent from class, so that in- and direct search will share the move_base_time
global move_base_time
move_base_time = 0.0

class MoveBase(smach.State):
    """
    Move the roboter base to given position.
    """

    def __init__(self):
        smach.State.__init__(
                self,
                outcomes=['succeeded', 'aborted'],
                input_keys=['goal_robot_pose'])

    @log
    @timed
    def execute(self, userdata): 
        rospy.loginfo('Executing MOVE_BASE')

	start = time()
        next_goal = MoveBaseGoal()
        next_goal.target_pose.header.frame_id = "/map"
        p = Pose(userdata.goal_robot_pose.position,
                  userdata.goal_robot_pose.orientation)
        next_goal.target_pose.pose = p 

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        if not client.wait_for_server(rospy.Duration.from_sec(10)):
            rospy.logwarn("Can't contact move_base action server")
            return 'aborted'

        rospy.loginfo('Sending goal to move_base action server.')
        move_result = client.send_goal_and_wait(next_goal)
        
        rospy.loginfo("Move_base action returned (3 being 'succeeded'): " + str(move_result))
        end = time()
        elapsed_time = end - start
        global move_base_time
        move_base_time += elapsed_time
        
        target_pose = next_goal.target_pose.pose
        rospy.loginfo("This has been the target robot pose, coming from NBV: " + str(target_pose))
        actual_pose = state_acquisition.get_robot_pose_cpp()
        rospy.loginfo("This is the actual robot pose after navigation: " + str(actual_pose))
        rospy.loginfo("This is the absolut difference: position:"
            + "\n  x: " + str(abs(target_pose.position.x - actual_pose.position.x))
            + "\n  y: " + str(abs(target_pose.position.y - actual_pose.position.y))
            + "\n  z: " + str(abs(target_pose.position.z - actual_pose.position.z))
            + "\norientation:"
            + "\n  x: " + str(abs(target_pose.orientation.x - actual_pose.orientation.x))
            + "\n  y: " + str(abs(target_pose.orientation.y - actual_pose.orientation.y))
            + "\n  z: " + str(abs(target_pose.orientation.z - actual_pose.orientation.z))
            + "\n  w: " + str(abs(target_pose.orientation.w - actual_pose.orientation.w)))
	
        rospy.loginfo("Move Base took : " + str(elapsed_time) + " seconds (Total time in move_base: " +  str(move_base_time) + ")")
        
        if move_result == 3:
            return 'succeeded'
        else:
            return 'aborted'


class FakeMoveBase(smach.State):
    """
    Sets robot to goal pose via '/initialpose' topic, bypassing actual ros navigation.
    """
    pub = None
    
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes=['succeeded'],
                input_keys=['goal_robot_pose'])
	self.pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)

    @log
    @timed
    def execute(self, userdata): 
        rospy.loginfo('Executing FAKE_MOVE_BASE')

	start = time()
        next_goal = PoseWithCovarianceStamped()
        next_goal.header.frame_id = "/map"
        p = Pose(userdata.goal_robot_pose.position,
                  userdata.goal_robot_pose.orientation)
        next_goal.pose.pose = p

        rospy.loginfo('Publish goal to /initialpose.')
	self.pub.publish(next_goal)
        rospy.sleep(0.5)
        end = time()
        elapsed_time = end - start
        global move_base_time
        move_base_time += elapsed_time

        target_pose = next_goal.pose.pose
        rospy.loginfo("This has been the target robot pose: " + str(target_pose))
        actual_pose = state_acquisition.get_robot_pose_cpp()
        rospy.loginfo("This is the actual robot pose after navigation: " + str(actual_pose))
        rospy.loginfo("This is the absolut difference: position:"
            + "\n  x: " + str(abs(target_pose.position.x - actual_pose.position.x))
            + "\n  y: " + str(abs(target_pose.position.y - actual_pose.position.y))
            + "\n  z: " + str(abs(target_pose.position.z - actual_pose.position.z))
            + "\norientation:"
            + "\n  x: " + str(abs(target_pose.orientation.x - actual_pose.orientation.x))
            + "\n  y: " + str(abs(target_pose.orientation.y - actual_pose.orientation.y))
            + "\n  z: " + str(abs(target_pose.orientation.z - actual_pose.orientation.z))
            + "\n  w: " + str(abs(target_pose.orientation.w - actual_pose.orientation.w)))

        rospy.loginfo("Move Base took : " + str(elapsed_time) + " seconds (Total time in move_base: " +  str(move_base_time) + ")")
        
        return 'succeeded'

class MovePTU(smach.State):
    """
    Moves the PTU to a given pose. A pose should consist of a pan and tilt value.
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted'],
            input_keys=['goal_ptu_position'])

    def ptu_callback(self, data):
        """Callback for ptu movements"""
        pass
        # print "Feedback while PTU is moving: " + str(data)

    @timed
    @log
    def execute(self, userdata):
        rospy.loginfo('Executing MOVE_PTU')

        client = actionlib.SimpleActionClient('ptu_controller_actionlib',
                                              PTUMovementAction)
        if not client.wait_for_server(rospy.Duration.from_sec(10)):
            rospy.logwarn("Could not connect to ptu action server")
            return 'aborted'

        ptu_goal = PTUMovementGoal()
        ptu_goal.target_joint.header.seq = 0
        ptu_goal.target_joint.name = ['pan', 'tilt']
        ptu_goal.target_joint.velocity = [0, 0]
        ptu_goal.target_joint.position = [userdata.goal_ptu_position[0], 
                                          userdata.goal_ptu_position[1]]

        pan = userdata.goal_ptu_position[0] * numpy.pi/180.0
        tilt = userdata.goal_ptu_position[1] * numpy.pi/180.0
        rospy.loginfo("Moving PTU to goal (" + str(pan) + ", " + str(tilt) + ')')

        ptu_result = client.send_goal_and_wait(ptu_goal)
        rospy.loginfo("PTU action returned (3 being 'succeeded'): " + str(ptu_result))

        if ptu_result == 3:
            return 'succeeded'
        else:
            return 'aborted'

class PTUPoseCorrection(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted'],
            input_keys=['goal_camera_pose'])

    @timed
    @log
    def execute(self, userdata):
        rospy.loginfo('Executing PTU_POSE_CORRECTION')

        targetPosition = userdata.goal_camera_pose.position
        targetOrientation = userdata.goal_camera_pose.orientation
        getRobotStateClass = state_acquisition.GetRobotState()

        try:
            rospy.wait_for_service('/asr_robot_model_services/CalculateCameraPoseCorrection', timeout=5)
            calculateCameraPoseCorrection = rospy.ServiceProxy(
                '/asr_robot_model_services/CalculateCameraPoseCorrection',
                CalculateCameraPoseCorrection)
            actual_state = getRobotStateClass.get_robot_state()

            rospy.loginfo('Robot state after navigation ' + str(actual_state))

            actual_pan = actual_state.pan * numpy.pi/180
            actual_tilt = actual_state.tilt * numpy.pi/180
            rospy.loginfo('Pan and tilt in rad: ' + str(actual_pan) + ", " + str(actual_tilt))

            ptuConfig = calculateCameraPoseCorrection(actual_state, targetOrientation, targetPosition)
        except (rospy.ServiceException, rospy.exceptions.ROSException), e:
            rospy.logwarn(str(e))
            return 'aborted'

        rospy.loginfo('Moving PTU to corrected goal (' + str(ptuConfig.pan) + ', ' + str(ptuConfig.tilt) + ')')

        client = actionlib.SimpleActionClient('ptu_controller_actionlib',
                                              PTUMovementAction)

        if not client.wait_for_server(rospy.Duration.from_sec(10)):
            rospy.logwarn("Could not connect to ptu action server")
            return 'aborted'

        pan = ptuConfig.pan * 180/numpy.pi
        tilt = ptuConfig.tilt * 180/numpy.pi

        ptu_goal = PTUMovementGoal()
        ptu_goal.target_joint.header.seq = 0
        ptu_goal.target_joint.name = ['pan', 'tilt']
        ptu_goal.target_joint.velocity = [0, 0]
        ptu_goal.target_joint.position = [pan, tilt]
        ptu_result = client.send_goal_and_wait(ptu_goal)
        rospy.loginfo("PTU action returned (3 being 'succeeded'): " + str(ptu_result))

        if ptu_result == 3:
            return 'succeeded'
        else:
            rospy.logerr("Failed to correct PTU-pose. Resuming with current pose...")
            return 'aborted'

