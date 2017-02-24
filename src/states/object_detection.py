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
import time
import tf
import move
import sys

from asr_msgs.msg import AsrObject, AsrViewport
from geometry_msgs.msg import (Pose, PoseWithCovariance, 
    PoseWithCovarianceStamped, Point, Quaternion, PoseStamped)
from world_model.srv import PushFoundObject, PushFoundObjectList, GetMissingObjectList, PushViewport
from evaluation_decorators import *
from object_detectors_manager import ObjectDetectorsManager
import state_acquisition

class ObjectDetection(smach.State):
    """
    Tries to detect given searched_object_types in current_camera_pose.
    If no current_camera_pose is given, it is determined via tf.
    """
    # Keeps actual AsrObjects
    #Required to transmit detected objects to other states.
    detected_objects = set()
    #Objects still searched (type and id).
    missing_pbd_typeAndId = []

    current_camera_pose = None

    global numberOfObjectDetectionRuns
    numberOfObjectDetectionRuns = 0
    global numberOfAllSearchedObjectTypes
    numberOfAllSearchedObjectTypes = 0

    def __init__(self):
        smach.State.__init__(
                self,
                outcomes=['found_objects', 'aborted', 'no_objects_found'],
                input_keys=['searched_object_types'],
                output_keys=['detected_objects'])
        self.isInitDone = 0

    def detection_callback(self, data):
        #Do not accept estimates from objects not being part of any searched scene
	if not any(typeAndId.type == data.type and typeAndId.identifier == data.identifier for typeAndId in self.missing_pbd_typeAndId):
	    rospy.loginfo("Object with id " + data.identifier + " and type " + data.type + " is not missing in any scene, we look for.")
	    return
	if not any(object.type == data.type and object.identifier == data.identifier for object in self.detected_objects):
	    #found object for the first time
	    rospy.loginfo("Received at least one estimate for object with type=" + data.type + " and id=" + data.identifier)
	if ( len(data.sampledPoses) == 0 ):
	    rospy.logerr("ObjectDetection got AsrObject message, not containing any poses.")
	    sys.exit(1)

	# frustum culling
	# Note that this is over simplified frustum culling just used to
	# filter some very bad outliers.
	# Here we just put a bounding box around the camera
	is_valid =  (abs(data.sampledPoses[0].pose.position.x) < 0.4 and
		      abs(data.sampledPoses[0].pose.position.y) < 0.4 and
		      data.sampledPoses[0].pose.position.z < self.fcp*1.1 and
		      data.sampledPoses[0].pose.position.z > self.ncp*0.9)
        if not is_valid:
	    rospy.loginfo("An object estimate for "+str(data.type)+" rejected, since classified as outlier.")
	    return

	# Construct FoundObject to push to world_model
	# objects are transformed to map frame in world_model
	rospy.logdebug("Pushing object estimate " + str(data.type) + " into return lists and world model lists.")
	self.detected_objects.add(data)

    def initNode(self):
        rospy.loginfo('Init OBJECT_DETECTION')
        self.isInitDone = 1
        self.searched_object_types = None
        try:
            self.ncp = rospy.get_param("/nbv/ncp")
            self.fcp = rospy.get_param("/nbv/fcp")
            self.fovx = rospy.get_param("/nbv/fovx")
            self.fovy = rospy.get_param("/nbv/fovy")
        except Exception, e:
            rospy.logwarn("Could not get parameters from nbv")
            self.ncp = 0.4
            self.fcp = 1.5
            self.fovx = 30
            self.fovy = 20

    @log
    @key_press
    @timed
    def execute(self, userdata):
        rospy.loginfo('Executing OBJECT_DETECTION')
        if (self.isInitDone == 0):
            self.initNode()

        # Return immediately if we have no objects to look for
        if not 'searched_object_types' in userdata:
            userdata.detected_objects = []
            return 'no_objects_found'
        if not userdata.searched_object_types:
            return 'no_objects_found'

        #Acquire current pose of camera from tf tree.
        self.current_camera_pose = state_acquisition.get_camera_pose_cpp()

        #Empty detection result list and missingTypeAndIds each time state is reached.
        self.detected_objects.clear()
        del self.missing_pbd_typeAndId[:]

        #check if searched_object_types is only one object type and transform to list
        if not isinstance(userdata.searched_object_types, list):
            self.searched_object_types = [userdata.searched_object_types]
        else:
            self.searched_object_types = userdata.searched_object_types

        #Push viewports to world_model
        try:
            rospy.wait_for_service('/env/world_model/push_viewport', timeout=5)
            push_viewport = rospy.ServiceProxy(
                '/env/world_model/push_viewport', PushViewport)
            viewport = AsrViewport()
            viewport.pose = self.current_camera_pose
            viewport.object_type_name_list = self.searched_object_types
            viewport.ncp = self.ncp
            viewport.fcp = self.fcp
            viewport.fovx = self.fovx
            viewport.fovy = self.fovy
            push_viewport(viewport)

        except (rospy.ServiceException, rospy.exceptions.ROSException), e:
            rospy.logwarn(str(e))
            rospy.logwarn("service exception from world_model")

        # Get all missing objects
        try:
            rospy.wait_for_service('/env/world_model/get_missing_object_list', timeout=5)
            get_missing_object_list =rospy.ServiceProxy(
                '/env/world_model/get_missing_object_list', GetMissingObjectList)
            self.missing_pbd_typeAndId = get_missing_object_list().missingObjects

        except (rospy.ServiceException, rospy.exceptions.ROSException), e:
            rospy.logwarn(str(e))
            rospy.logwarn("service exception from world_model")
            return 'no_objects_found'

        # We need to filter the search_objects in random/cropbox search mode
        # cropbox record doesn't use object_detection
        mode = rospy.get_param("/scene_exploration_sm/mode");
        if mode == 4 or mode == 5:
            self.searched_object_types = set()
            for obj in self.missing_pbd_typeAndId:
                self.searched_object_types.add(str(obj.type))

        global numberOfObjectDetectionRuns
        global numberOfAllSearchedObjectTypes
        numberOfObjectDetectionRuns += 1
        numberOfAllSearchedObjectTypes += len(self.searched_object_types)
        rospy.loginfo("Number of object detections: " + str(numberOfObjectDetectionRuns))
        rospy.loginfo("Number of all searched_object_types: " + str(numberOfAllSearchedObjectTypes))
        rospy.loginfo("Average number of searched_object_types per run: " + str((1.0 * numberOfAllSearchedObjectTypes / numberOfObjectDetectionRuns)))

        detectors_manager = ObjectDetectorsManager()
        if detectors_manager.start_recognizers(self.searched_object_types) is 'aborted':
            return 'aborted'
        # wait till object detectors will be started
        rospy.sleep(2.0)
        # Listen for 'detection_duration * number_of_searched_object_types' seconds for detected objects
        rospy.loginfo("All requested recognizers should now publish to " + "/stereo/objects")
        wait_time = rospy.get_param("/nbv/speedFactorRecognizer")*len(self.searched_object_types)

        future = time() + wait_time
        sub = rospy.Subscriber('/stereo/objects',
                               AsrObject,
                               self.detection_callback)
        while (time() < future):
            pass
        # unregister, otherwise the callbacks are still called
        sub.unregister()
 
        if detectors_manager.stop_recognizers(self.searched_object_types) is 'aborted':
            rospy.loginfo("stop recog send aborted")
            return 'aborted'
    
        rospy.loginfo("All recognizers are shut down.")

        if len(self.detected_objects) == 0:
            userdata.detected_objects = list(self.detected_objects)
            return 'no_objects_found'

        # Push to found objects in world model     
        rospy.loginfo("Pushing the following estimates to world_model.")
        try:
            for output_object in self.detected_objects:
                rospy.loginfo("Got object estimate with type " + output_object.type + " and id " + output_object.identifier + " provided by " + output_object.providedBy)
                rospy.loginfo("Its position is "
                              + "x: " + str(output_object.sampledPoses[0].pose.position.x)
                              + " y: " + str(output_object.sampledPoses[0].pose.position.y)
                              + " z: " + str(output_object.sampledPoses[0].pose.position.z) + " and its orientation is "
                              + "x: " + str(output_object.sampledPoses[0].pose.orientation.x)
                              + " y: " + str(output_object.sampledPoses[0].pose.orientation.y)
                              + " z: " + str(output_object.sampledPoses[0].pose.orientation.z)
                              + " w: " + str(output_object.sampledPoses[0].pose.orientation.w))

            rospy.wait_for_service('/env/world_model/push_found_object_list',
                                   timeout=3)
            push_found_objects = rospy.ServiceProxy(
                '/env/world_model/push_found_object_list', PushFoundObjectList)
            push_found_objects(list(self.detected_objects))
        except (rospy.exceptions.ROSException, rospy.ServiceException) as e:
            rospy.logwarn("Could not find world_model service")
            return 'aborted'

        userdata.detected_objects = list(self.detected_objects)
        return 'found_objects'
