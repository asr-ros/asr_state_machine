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
import __builtin__
import tf
import math
import os.path

from lxml import etree

from geometry_msgs.msg import Pose, Point, Quaternion
import common.state_acquisition as state_acquisition
from common.evaluation_decorators import *

class CropboxStateRecording(smach.State):
    """
    Please enter some comments here!
    """

    def __init__(self):
        smach.State.__init__(
                self,
                outcomes=['succeeded', 'aborted'],
                input_keys=['goal_camera_pose',
                            'goal_robot_pose',
                            'goal_ptu_position',
                            'deactivated_object_normals_count'])
        self.isInitDone = 0


    def initNode(self):
        rospy.loginfo('Init STATE_RECORDING')
        self.isInitDone = 1

        self.cropBoxRecordingPath = rospy.get_param('/scene_exploration_sm/CropBoxRecordingPath')
        rospy.loginfo("CropBoxRecordingPath: " + str(self.cropBoxRecordingPath))
        initFile(self.cropBoxRecordingPath)


        self.xmlTemplate = """<state>
        <goal_camera_pose>
            <position x="%(goal_camera_pose.position.x)s" y="%(goal_camera_pose.position.y)s" z="%(goal_camera_pose.position.z)s"/>
            <orientation x="%(goal_camera_pose.orientation.x)s" y="%(goal_camera_pose.orientation.y)s" z="%(goal_camera_pose.orientation.z)s" w="%(goal_camera_pose.orientation.w)s" />
        </goal_camera_pose>
        <goal_robot_pose>
            <position x="%(goal_robot_pose.position.x)s" y="%(goal_robot_pose.position.y)s" z="%(goal_robot_pose.position.z)s"/>
            <orientation x="%(goal_robot_pose.orientation.x)s" y="%(goal_robot_pose.orientation.y)s" z="%(goal_robot_pose.orientation.z)s" w="%(goal_robot_pose.orientation.w)s" />
        </goal_robot_pose>
        <goal_ptu_position pan="%(goal_ptu_position.pan)s" tilt="%(goal_ptu_position.tilt)s" />
        <deactivated_object_normals_count count="%(deactivated_object_normals_count)s" />
    </state>"""


    @log
    @key_press
    @timed
    def execute(self, userdata):
        rospy.loginfo('Executing STATE_RECORDING')
        if (self.isInitDone == 0):
            if self.initNode() == 'aborted':
                return 'aborted'

        data = {'goal_camera_pose.position.x':str(userdata.goal_camera_pose.position.x),
                'goal_camera_pose.position.y':str(userdata.goal_camera_pose.position.y),
                'goal_camera_pose.position.z':str(userdata.goal_camera_pose.position.z),
                'goal_camera_pose.orientation.x':str(userdata.goal_camera_pose.orientation.x),
                'goal_camera_pose.orientation.y':str(userdata.goal_camera_pose.orientation.y),
                'goal_camera_pose.orientation.z':str(userdata.goal_camera_pose.orientation.z),
                'goal_camera_pose.orientation.w':str(userdata.goal_camera_pose.orientation.w),
                'goal_robot_pose.position.x':str(userdata.goal_robot_pose.position.x),
                'goal_robot_pose.position.y':str(userdata.goal_robot_pose.position.y),
                'goal_robot_pose.position.z':str(userdata.goal_robot_pose.position.z),
                'goal_robot_pose.orientation.x':str(userdata.goal_robot_pose.orientation.x),
                'goal_robot_pose.orientation.y':str(userdata.goal_robot_pose.orientation.y),
                'goal_robot_pose.orientation.z':str(userdata.goal_robot_pose.orientation.z),
                'goal_robot_pose.orientation.w':str(userdata.goal_robot_pose.orientation.w),
                'goal_ptu_position.pan':str(userdata.goal_ptu_position[0]),
                'goal_ptu_position.tilt':str(userdata.goal_ptu_position[1]),
                'deactivated_object_normals_count':str(userdata.deactivated_object_normals_count)}

        addXml(self.cropBoxRecordingPath, self.xmlTemplate%data)

        return 'succeeded'


class GridInitStateRecording(smach.State):
    """
    Please enter some comments here!
    """

    def __init__(self):
        smach.State.__init__(
                self,
                outcomes=['succeeded', 'aborted'],
                input_keys=['goal_robot_pose',
                            'goal_ptu_position',])
        self.isInitDone = 0


    def initNode(self):
        rospy.loginfo('Init STATE_RECORDING')
        self.isInitDone = 1

        self.initializedGridFilePath = rospy.get_param('/scene_exploration_sm/initializedGridFilePath')
        rospy.loginfo("initializedGridFilePath: " + str(self.initializedGridFilePath))
        initFile(self.initializedGridFilePath)


        self.xmlTemplate = """<state>
        <goal_camera_pose>
            <position x="%(goal_camera_pose.position.x)s" y="%(goal_camera_pose.position.y)s" z="%(goal_camera_pose.position.z)s"/>
            <orientation x="%(goal_camera_pose.orientation.x)s" y="%(goal_camera_pose.orientation.y)s" z="%(goal_camera_pose.orientation.z)s" w="%(goal_camera_pose.orientation.w)s" />
        </goal_camera_pose>
        <goal_robot_pose>
            <position x="%(goal_robot_pose.position.x)s" y="%(goal_robot_pose.position.y)s" z="%(goal_robot_pose.position.z)s"/>
            <orientation x="%(goal_robot_pose.orientation.x)s" y="%(goal_robot_pose.orientation.y)s" z="%(goal_robot_pose.orientation.z)s" w="%(goal_robot_pose.orientation.w)s" />
        </goal_robot_pose>
        <goal_ptu_position pan="%(goal_ptu_position.pan)s" tilt="%(goal_ptu_position.tilt)s" />
    </state>"""


    @log
    @key_press
    @timed
    def execute(self, userdata):
        rospy.loginfo('Executing STATE_RECORDING')
        if (self.isInitDone == 0):
            if self.initNode() == 'aborted':
                return 'aborted'

        current_camera_pose = state_acquisition.get_camera_pose_cpp()

        data = {'goal_camera_pose.position.x':str(current_camera_pose.position.x),
                'goal_camera_pose.position.y':str(current_camera_pose.position.y),
                'goal_camera_pose.position.z':str(current_camera_pose.position.z),
                'goal_camera_pose.orientation.x':str(current_camera_pose.orientation.x),
                'goal_camera_pose.orientation.y':str(current_camera_pose.orientation.y),
                'goal_camera_pose.orientation.z':str(current_camera_pose.orientation.z),
                'goal_camera_pose.orientation.w':str(current_camera_pose.orientation.w),
                'goal_robot_pose.position.x':str(userdata.goal_robot_pose.position.x),
                'goal_robot_pose.position.y':str(userdata.goal_robot_pose.position.y),
                'goal_robot_pose.position.z':str(userdata.goal_robot_pose.position.z),
                'goal_robot_pose.orientation.x':str(userdata.goal_robot_pose.orientation.x),
                'goal_robot_pose.orientation.y':str(userdata.goal_robot_pose.orientation.y),
                'goal_robot_pose.orientation.z':str(userdata.goal_robot_pose.orientation.z),
                'goal_robot_pose.orientation.w':str(userdata.goal_robot_pose.orientation.w),
                'goal_ptu_position.pan':str(userdata.goal_ptu_position[0]),
                'goal_ptu_position.tilt':str(userdata.goal_ptu_position[1])}


        addXml(self.initializedGridFilePath, self.xmlTemplate%data)
        return 'succeeded'


def initFile(path):
    if os.path.isfile(path):
        rospy.loginfo("File " + path + " does exists and will be erased.")
        open(path, 'w').close()

    with open(path, "w+") as xmlFile:
        xmlFile.write("""<root></root>""")

def addXml(path, xmlToAdd):
    parser = etree.XMLParser(remove_blank_text=True)
    tree = etree.parse(path, parser)

    root = tree.getroot()
    new_state = etree.XML(xmlToAdd, parser)
    root.append(new_state)
    # Make pretty XML
    tree._setroot(etree.fromstring(etree.tostring(root, encoding='UTF-8', pretty_print=True)))

    tree.write(path)
