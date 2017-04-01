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
import dynamic_reconfigure.client

from asr_msgs.msg import AsrAttributedPointCloud, AsrAttributedPoint
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import OccupancyGrid

from common.evaluation_decorators import *
from asr_world_model.srv import GetIntermediateObjectWeight, GetRecognizerName

class CropBoxGeneration(smach.State):
    """
    Please enter some comments here!
    """

    def __init__(self):
        smach.State.__init__(
                self,
                outcomes=['succeeded', 'aborted'],
                output_keys=['object_pointcloud'])
        self.isInitDone = 0
        # You can activate the params for dynamic reconfigure of the NBV here
        self.newMOmegaUtility = None #320
        self.newMOmegaBase = None #50


    def initNode(self):
        rospy.loginfo('Init CROP_BOX_GENERATION')
        self.isInitDone = 1
        self.isWaitForMap = 0
        self.isReconfigDone = 0
        rospy.Subscriber("map", OccupancyGrid, self.mapCallback)
        self.reconfigureNBV()
        self.initSteps()

        self.objectType = rospy.get_param('/scene_exploration_sm/PlaceholderObjectTypeInCropboxSearch')
        self.identifier = rospy.get_param('/scene_exploration_sm/PlaceholderIdentifierInCropboxSearch')
        rospy.loginfo(str(self.objectType) + " with identifier " + str(self.identifier) + " will be used for CropBoxGeneration.")

        r = rospy.Rate(20) # 20hz
        rospy.loginfo('Wait for callback from map and reconfiguration of NBV')
        while (self.isWaitForMap == 0 or self.isReconfigDone == 0):
            r.sleep()

    def mapCallback(self, data):
        self.width = (data.info.width * data.info.resolution) / 2 + 0.48
        self.length = (data.info.height * data.info.resolution) / 2 + 0.35
        self.height = rospy.get_param("/scene_exploration_sm/RoomHeight")
        rospy.loginfo('Points in point_cloud are generated between width: ' + str(-self.width) + ' x ' + str(self.width)
            + ', length: ' + str(-self.length) + ' x ' + str(self.length) + ' and height: ' + str(0) + ' x ' + str(self.height))
        self.isWaitForMap = 1

    def initSteps(self):
        fcp = rospy.get_param("/nbv/fcp")
        ncp = rospy.get_param("/nbv/ncp")
      
        fovx = rospy.get_param("/nbv/fovx")
        self.groudStep = min((fcp - ncp) * math.tan(math.radians(fovx) / 2.0), (fcp - ncp))

        fovy = rospy.get_param("/nbv/fovy")
        self.heightStep = (fcp - ncp) * math.tan(math.radians(fovy) / 2.0)
        rospy.loginfo("Interval of points in point_cloud on groud: " + str(self.groudStep) + " and in height: " + str(self.heightStep))


    def reconfigureNBVCallback(self, config):
        if ((self.newMOmegaUtility is None or config.groups.groups.Rating.mOmegaUtility == self.newMOmegaUtility) and
                (self.newMOmegaBase is None or config.groups.groups.Rating.mOmegaBase == self.newMOmegaBase) and
                config.groups.groups.Other.enableCropBoxFiltering == True and
                config.groups.groups.Other.enableIntermediateObjectWeighting == False):
            rospy.loginfo("Dynamic reconfiguration of NBV done.")
            self.isReconfigDone = 1

    def reconfigureNBV(self):
        mapToSet = {"enableCropBoxFiltering": True, "enableIntermediateObjectWeighting": False}
        rospy.loginfo("Dynamic reconfigure of NBV: enableCropBoxFiltering=True and enableIntermediateObjectWeighting=False")

        if self.newMOmegaUtility is not None:
            rospy.loginfo("Dynamic reconfigure of NBV: mOmegaUtility=" + str(self.newMOmegaUtility))
            mapToSet.update({"mOmegaUtility": self.newMOmegaUtility})
        if self.newMOmegaBase is not None:
            rospy.loginfo("Dynamic reconfigure of NBV: mOmegaBase=" + str(self.newMOmegaBase))
            mapToSet.update({"mOmegaBase": self.newMOmegaBase})

        client = dynamic_reconfigure.client.Client("nbv", timeout=30, config_callback=self.reconfigureNBVCallback)
        client.update_configuration(mapToSet)


    @log
    @key_press
    @timed
    def execute(self, userdata):
        rospy.loginfo('Executing CROP_BOX_GENERATION')
        if (self.isInitDone == 0):
            if self.initNode() == 'aborted':
                return 'aborted'

        point_cloud = AsrAttributedPointCloud()

        for x in self.frange(-self.width, self.width, self.groudStep):
            for y in self.frange(-self.length, self.length, self.groudStep):
                for z in self.frange(0.0, self.height, self.heightStep):
                        point = AsrAttributedPoint()
                        point.pose.position = Point(*[x, y, z])
                        point.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(-1.57,0,0))
                        point.type = self.objectType
                        point.identifier = self.identifier
                        point_cloud.elements.append(point)
        userdata.object_pointcloud = point_cloud
        return 'succeeded'


    def frange(self, start, stop, step):
         i = start
         while i <= stop:
             yield i
             i += step

