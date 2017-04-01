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
import random
import tf
import common.state_acquisition as state_acquisition

from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid
from asr_flir_ptu_driver.srv import Range
from asr_robot_model_services.srv import IsPositionAllowed

from common.evaluation_decorators import *


class GenerateRandomPose(smach.State):

    class RobotState:

        def __init__(self):
           self.pose = Pose()
           self.tilt = 0.0

        def __repr__(self):
          return str(self.pose) + "\ntilt: " + str(self.tilt)


    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'],
                             output_keys=['goal_robot_pose', 'goal_ptu_position'])
        self.searched_robot_states = []
        self.isInitDone = 0
        self.numberOfGeneratedPoses = 0
        self.amountOfTime = 0
        self.attempts = 0

    def initNode(self):
        rospy.loginfo('Init GENERATE_RANDOM_POSE')
        self.isInitDone = 1
        self.isWaitForMap = 0
        rospy.Subscriber("map", OccupancyGrid, self.mapCallback)

        #Increase recognizer frequency in random search ONLY to speed up search.
        rospy.set_param("/nbv/speedFactorRecognizer", 0.5)

        self.initTiltRange()
        self.AllowSameViewInRandomSearch = rospy.get_param("/scene_exploration_sm/AllowSameViewInRandomSearch", False)
        rospy.loginfo("AllowSameViewInRandomSearch has value: " + str(self.AllowSameViewInRandomSearch))
        self.initThresholds()

        r = rospy.Rate(20) # 20hz
        rospy.loginfo('Wait for callback from map')
        while (self.isWaitForMap == 0):
            r.sleep()

    def mapCallback(self, data):
        self.width = (data.info.width * data.info.resolution) / 2 + 0.5
        self.height = (data.info.height * data.info.resolution) / 2 + 0.5
        rospy.loginfo('Random poses are generated between width: ' + str(-self.width) + ' x ' + str(self.width)
            + ' and height: ' + str(-self.height) + ' x ' + str(self.height))
        self.isWaitForMap = 1

    def initTiltRange(self):
        try:
            rospy.wait_for_service('/asr_flir_ptu_driver/get_range')
            ptuRange = rospy.ServiceProxy('/asr_flir_ptu_driver/get_range', Range)()
            self.tiltMin = ptuRange.tilt_min_angle
            self.tiltMax = ptuRange.tilt_max_angle
            rospy.loginfo('Random tilts are generated between tiltMin: ' + str(self.tiltMin) + ' and tiltMax: ' + str(self.tiltMax))
        except (rospy.ServiceException, rospy.exceptions.ROSException), e:
            rospy.logwarn(str(e))

    def initThresholds(self):
        self.positionThreshold = rospy.get_param("/nbv/global_costmap/robot_radius")

        fovx = rospy.get_param("/nbv/fovx")
        self.orientationThreshold = fovx / 2.0

        fovy = rospy.get_param("/nbv/fovy")
        self.tiltThreshold = fovy / 2.0
        rospy.loginfo("positionThreshold: " + str(self.positionThreshold) + "; orientationThreshold: " + str(self.orientationThreshold)
            + "; tiltThreshold: " + str(self.tiltThreshold))

    @log
    @key_press
    @timed
    def execute(self, userdata):
        rospy.loginfo('Executing GENERATE_RANDOM_POSE')
        beginn = rospy.get_time()
        if (self.isInitDone == 0):
            self.initNode()

        random_robot_state = self.RobotState()
        while True:
            random_robot_state.pose.position = Point(*[random.uniform(-self.width, self.width), random.uniform(-self.height, self.height), 0])
            random_robot_state.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, random.uniform(0,359)))
            random_robot_state.tilt = random.uniform(self.tiltMin, self.tiltMax)

            self.attempts += 1
            if self.checkCalculatedStateAllowed(random_robot_state):
                break;

        self.searched_robot_states.append(random_robot_state)

        userdata.goal_robot_pose = random_robot_state.pose
        # pan is const, because it is redundant with orientation
        random_ptu_config = [0, random_robot_state.tilt]
        userdata.goal_ptu_position = random_ptu_config

        rospy.loginfo('Generated random\n' + str(random_robot_state.pose))
        rospy.loginfo('Generated random PTU (tilt): ' + str(random_ptu_config))

        self.numberOfGeneratedPoses += 1
        self.amountOfTime += (rospy.get_time() - beginn)
        rospy.loginfo('Average number of attempts for isPosReachable: ' + str(self.attempts / float(self.numberOfGeneratedPoses)) + ' for all '
            + str(self.numberOfGeneratedPoses) + ' generated poses to finish')
        rospy.loginfo('GENERATE_RANDOM_POSE took ' + str(self.amountOfTime / float(self.numberOfGeneratedPoses)) + ' secs average to finish')
        return 'succeeded'

    def checkCalculatedStateAllowed(self, robot_state_to_test):
        if self.isRobotPositionAllowed(robot_state_to_test.pose.position) is False:
            return False
        if self.AllowSameViewInRandomSearch is True:
            return True

        for oldRobotState in self.searched_robot_states:
            if state_acquisition.approx_equal_robot_state(oldRobotState.pose, oldRobotState.tilt, robot_state_to_test.pose,
                    robot_state_to_test.tilt, self.positionThreshold, self.orientationThreshold, self.tiltThreshold):
                return False
        return True

    def isRobotPositionAllowed(self, robotPosition):
        """
        returns if robotPosition is allowed using the appropriate service call
        """
        rospy.wait_for_service('/asr_robot_model_services/IsPositionAllowed')
        try:
            isReachableHandler = rospy.ServiceProxy('/asr_robot_model_services/IsPositionAllowed', IsPositionAllowed)
            ret = isReachableHandler(robotPosition)
            return ret.isAllowed
        except (rospy.ServiceException, rospy.exceptions.ROSException), e:
            rospy.logwarn(str(e))

