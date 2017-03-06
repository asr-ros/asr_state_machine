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
import imp

from asr_recognizer_prediction_ism.srv import FindScenes, GetPointCloud
from asr_msgs.msg import AsrObject, AsrTypeAndId
from asr_msgs.msg import AsrAttributedPointCloud, AsrAttributedPoint
from asr_world_model.msg import CompletePattern
from asr_world_model.srv import GetFoundObjectList, GetMissingObjectList, GetCompletePatterns

from evaluation_decorators import *


"""
Contains states for pose prediction and scene recognition.  Both states use
service calls to the ism node in the recognizer_predictin_ism package.
"""

class SceneRecognition(smach.State):
    """
    Tries to recognize scene with the objects saved in the world model.
    Active scene recognition finishes, once all object in all scenes are found. Otherwise the pose prediction
    in the ism node receives all found scenes and this state returns with
    'found_scenes'.

    Note that we don't get the actual scene recognition results any other format
    than a serialized stream.
    """

    def __init__(self):
        smach.State.__init__(
                self,
                outcomes=['found_scenes', 'aborted', 'found_all_objects', 'found_all_required_scenes'])
        self.isInitDone = 0


    def initNode(self):
        rospy.loginfo('Init SceneRecognition')
        self.isInitDone = 1
        dbfilename = rospy.get_param('/rp_ism_node/dbfilename');
        dbPathSplit = dbfilename.split("/")
        dbName = dbPathSplit[-1]

        path = rospy.get_param('/scene_exploration_sm/CompletePatternsPath')
        path = path.replace("XXX", dbName);
        rospy.loginfo("Parsed CompletePatternsPath: " + str(path))

        try:
            self.evaluateCompletePatternsModule = imp.load_source("evaluateCompletePatterns", path)
        except IOError:
            rospy.logwarn("No CompletePatternsFile found -> no abortion through found_all_required_scenes")
            self.evaluateCompletePatternsModule = None

    @log
    @key_press
    @timed
    def execute(self, userdata):
        rospy.loginfo('Executing SCENE_RECOGNITION')
        if (self.isInitDone == 0):
            if self.initNode() == 'aborted':
                return 'aborted'

        # Get all found objects from world_model
        detected_objects = []

        try:
            rospy.wait_for_service('/env/asr_world_model/get_found_object_list', timeout=5)
            get_detected_objects =rospy.ServiceProxy(
                '/env/asr_world_model/get_found_object_list', GetFoundObjectList)
            detected_objects_response = get_detected_objects()
            detected_objects = detected_objects_response.object_list
            rospy.loginfo("World model returns " + str(len(detected_objects)) + " found objects:")
            # extract asr_object and print found objects
            for obj in detected_objects:
                rospy.loginfo("["+ obj.type + ", " + obj.identifier + "]")
        except (rospy.ServiceException, rospy.exceptions.ROSException), e:
            rospy.logwarn(str(e))
            rospy.logwarn("service exception from world_model")
            return 'aborted'
     
        rospy.loginfo("Now calling find_scenes service for all found objects.")
        # find scenes with objects from world_model
        try:
            rospy.wait_for_service('/rp_ism_node/find_scenes', timeout=5)
        except rospy.exceptions.ROSException, e:
            rospy.logwarn("Could not find find_scenes service")
            return 'aborted'

        try:
            scene_recognizer = rospy.ServiceProxy(
                '/rp_ism_node/find_scenes',
                FindScenes)
            scene_results = scene_recognizer(detected_objects)

            rospy.loginfo("Response of service call: " + str(scene_results))
            if hasattr(__builtin__, 'log_dir'):
                with open(__builtin__.log_dir +  '/scene_results.txt', 'a') as log:
                    log.write(str(scene_results))
                    log.write("========================================================\n")

            try:
                rospy.wait_for_service('/env/asr_world_model/get_missing_object_list', timeout=5)
                get_found_all =rospy.ServiceProxy(
                    '/env/asr_world_model/get_missing_object_list', GetMissingObjectList)
                get_missing_object_list_response = get_found_all()
            except (rospy.ServiceException, rospy.exceptions.ROSException), e:
                rospy.logwarn(str(e))
                rospy.logwarn("service exception from world_model")
                return 'aborted'

            rospy.loginfo('Objects still not found: ')
            for missing in get_missing_object_list_response.missingObjects:
                rospy.loginfo('[' + missing.type + ', ' + missing.identifier + ']')
            if len(get_missing_object_list_response.missingObjects) == 0 :
                return 'found_all_objects'

            if self.checkAllRequiredScenesWhereFound() is True:
                return 'found_all_required_scenes'
            else:
                return 'found_scenes'

        except rospy.ServiceException, e:
            rospy.logwarn("ServiceException on calling find_scenes")
            rospy.logwarn(e)
            return 'aborted'


    def checkAllRequiredScenesWhereFound(self):
        if self.evaluateCompletePatternsModule is None:
            return False

        rospy.wait_for_service('/env/asr_world_model/get_complete_patterns', timeout=5)
        get_complete_patterns_list = rospy.ServiceProxy('/env/asr_world_model/get_complete_patterns', GetCompletePatterns)
        complete_patterns = get_complete_patterns_list().completePatterns
        rospy.loginfo("Got complete_patterns from world_model:\n" + str(complete_patterns))

        result = self.evaluateCompletePatternsModule.evaluateCompletePatterns(complete_patterns)
        rospy.loginfo("Result of evaluateCompletePatterns: " + str(result))
        return result

class PosePrediction(smach.State):
    """
    Generates AttributedPointCloud for scene hypotheses resampled by particle filter in rp_ism_node. The scene
    hypotheses (all from scene recognition) for resampling is not passed in the userdata, but directly inside the
    asr_recognizer_prediction_ism package. We get into this state if the scene recognition
    recognized at least one scene.
    """

    def __init__(self):
        smach.State.__init__(
                self,
                outcomes=['succeeded', 'aborted', 'no_predictions_left'],
                output_keys=['object_pointcloud'])
    
    @log
    @key_press
    @timed
    def execute(self, userdata):
        rospy.loginfo('Executing OBJECT_POSE_PREDICTION')

        rospy.loginfo("Now calling get_point_cloud service to get predicted object poses.")
        try:
            rospy.wait_for_service('/rp_ism_node/get_point_cloud',
                                   timeout=5)
        except rospy.exceptions.ROSException, e:

            rospy.logwarn(e)
            return 'aborted'

        # Get all found objects from world_model
        detected_objects = []

        try:
            rospy.wait_for_service('/env/asr_world_model/get_found_object_list', timeout=5)
            get_detected_objects =rospy.ServiceProxy(
                '/env/asr_world_model/get_found_object_list', GetFoundObjectList)
            detected_objects_response = get_detected_objects()
            detected_objects = detected_objects_response.object_list
            rospy.loginfo("World model returns " + str(len(detected_objects)) + " found objects:")
            # extract asr_object and print found objects
            for obj in detected_objects:
                rospy.loginfo("["+ obj.type + ", " + obj.identifier + "]")
        except (rospy.ServiceException, rospy.exceptions.ROSException), e:
            rospy.logwarn(str(e))
            rospy.logwarn("service exception from world_model")
            return 'aborted'

        try:
            pose_predictor = rospy.ServiceProxy(
                '/rp_ism_node/get_point_cloud',
                GetPointCloud)
            pose_prediction = pose_predictor(detected_objects)
            result_pointcloud = pose_prediction.point_cloud

            rospy.loginfo("Response of service call (pointcloud omitted): " + str(pose_prediction.output))
            if hasattr(__builtin__, 'log_dir'):
                with open(__builtin__.log_dir +  '/prediction_results.txt', 'a') as log:
                    log.write(str(pose_prediction.output))
                    log.write("========================================================\n")

            if len(result_pointcloud.elements) == 0:
                # TODO better abort criterion
                rospy.loginfo("Empty pointcloud from pose prediction")
                return 'no_predictions_left'

            userdata.object_pointcloud = result_pointcloud
            return 'succeeded'
        except rospy.ServiceException, e:
            rospy.logwarn("Service exception on calling get_point_cloud")
            rospy.logwarn(str(e))
            return 'aborted'




