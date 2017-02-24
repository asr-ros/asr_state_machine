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
import types
import states.init
import states.state_acquisition as state_acquisition
import xml.etree.ElementTree as ET

from actionlib import SimpleActionClient
from actionlib.msg import actionlib
from asr_direct_search_manager.msg import direct_search_manager
from next_best_view.srv import TriggerFrustumVisualization
from world_model.srv import GetMissingObjectList, GetAllObjectsList
from asr_msgs.msg import AsrTypeAndId

from evaluation_decorators import *

"""
This file defines useful states for usage with the MILD infrastructure. Keep in
mind that most of them need some sort of userdata that must be provided during
runtime in order to behave properly.
"""


class CheckSearchFinished(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['search_finished', 'search_not_finished', 'aborted'],
                             output_keys=['searched_object_types_and_ids'])

    @log
    @key_press
    @timed
    def execute(self, userdata):
        rospy.loginfo('Executing CHECK_SEARCH_FINISHED')

        searched_object_types_and_ids = get_search_object_types_and_ids()
        if (searched_object_types_and_ids == 'aborted'):
            return 'aborted'
        userdata.searched_object_types_and_ids = searched_object_types_and_ids

        if rospy.get_param("/scene_exploration_sm/StopAtfirst"):
            return 'search_finished'
        elif (not rospy.get_param("/scene_exploration_sm/SearchAllobject")
                and len(searched_object_types_and_ids) is 0):
            return 'search_finished'
        else:
            return 'search_not_finished'


class DirectSearchInit(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['aborted', 'object_poses_from_demonstrations_given',
                                             'no_object_poses_from_demonstrations'],
                             output_keys=['searched_object_types_and_ids'])

    @log
    @key_press
    @timed
    def execute(self, userdata):
        rospy.loginfo('Executing DIRECT_SEARCH_INIT')

        mode = rospy.get_param('/scene_exploration_sm/mode')
        if mode != 3:
            states.init.clear_costmap()
            states.init.clear_world_model()
            states.init.reset_ism()
            result = execute_direct_search_action("Reset")
            if (result == 'aborted'):
                return 'aborted'

        searched_object_types_and_ids = get_search_object_types_and_ids()
        if (searched_object_types_and_ids == 'aborted'):
            return 'aborted'
        userdata.searched_object_types_and_ids = searched_object_types_and_ids

        result = execute_direct_search_action("BackToInitial", searched_object_types_and_ids)
        if (result == 'aborted'):
            return 'aborted'

        if result.arePosesFromDemonstrationLeft:
            return 'object_poses_from_demonstrations_given'
        else:
            return 'no_object_poses_from_demonstrations'


class GetGoalCameraPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['MoveRobot', 'MovePTUOnly', 'nowhere_left_to_search', 'aborted'],
                                   output_keys=['goal_camera_pose', 'goal_robot_pose', 'goal_ptu_position', 'filtered_searched_object_types'],
                                   input_keys=['filtered_searched_object_types', 'searched_object_types_and_ids'])
        self.arePosesFromDemonstrationLeft = False
        self.werePosesFromDemonstrationLeftEver = False


    @log
    @key_press
    @timed
    def execute(self, userdata):
        rospy.loginfo('Executing GET_GOAL_CAMERA_POSE')

        if not self.arePosesFromDemonstrationLeft and self.werePosesFromDemonstrationLeftEver:
            return 'nowhere_left_to_search'

        result = execute_direct_search_action("GetGoalCameraPose", userdata.searched_object_types_and_ids)
        if (result == 'aborted'):
            return 'aborted'

        if result.arePosesFromDemonstrationLeft:
            self.werePosesFromDemonstrationLeftEver = True

        self.arePosesFromDemonstrationLeft = result.arePosesFromDemonstrationLeft

        if result.isNoPoseLeft:
            return 'nowhere_left_to_search'

        #Trigger old frustum viz for current_camera_pose
        current_camera_pose = state_acquisition.get_camera_pose_cpp()
        trigger_old_frustum_viz(current_camera_pose)
        trigger_frustum_viz(result.goalCameraPose)
        userdata.goal_camera_pose = result.goalCameraPose
        userdata.goal_ptu_position = [result.pan, result.tilt]

        userdata.filtered_searched_object_types = []
        for object_type_and_id in result.filteredSearchedObjectTypesAndIds:
            userdata.filtered_searched_object_types.append(object_type_and_id.type)

        rospy.loginfo("Searched object types and ids:\n" + str(userdata.searched_object_types_and_ids))
        rospy.loginfo("Filtered searched object types and ids:\n" + str(result.filteredSearchedObjectTypesAndIds))

        if result.isSameRobotPoseAsBefore:
            return 'MovePTUOnly'
        else:
            userdata.goal_robot_pose = result.goalRobotPose
            return 'MoveRobot'


class FrustumViz(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    @log
    @key_press
    @timed
    def execute(self, userdata):
        rospy.loginfo('Executing FRUSTUM_VIZ')

        # Sometimes the PTU has not moved yet
        rospy.sleep(0.2)
        camera_pose = state_acquisition.get_camera_pose_cpp()
        trigger_frustum_viz(camera_pose)
        return 'succeeded'


def execute_direct_search_action(action_command, searched_object_types_and_ids=None):
    rospy.loginfo("Direct search manager service call for: " + str(action_command))
    client = actionlib.SimpleActionClient('direct_search_manager', asr_direct_search_manager.msg.directSearchAction)
    client.wait_for_server()
    goal = asr_direct_search_manager.msg.directSearchGoal(command=str(action_command), searchedObjectTypesAndIds=searched_object_types_and_ids)
    client.send_goal(goal)
    client.wait_for_result()
    if client.get_state() != 3:
        rospy.logerr("Direct search manager service call for: " + str(client.get_state()) + " was not successful!")
        return 'aborted'
    return client.get_result()


def trigger_old_frustum_viz(current_camera_pose):
    try:
        rospy.wait_for_service('/nbv/trigger_old_frustum_visualization', timeout=5)
        trigger_old_frustum_v = rospy.ServiceProxy(
                '/nbv/trigger_old_frustum_visualization', TriggerFrustumVisualization)
        trigger_old_frustum_v(current_camera_pose)
    except (rospy.ServiceException, rospy.exceptions.ROSException), e:
        rospy.logwarn(str(e))

def trigger_frustum_viz(camera_pose):
    try:
        rospy.wait_for_service('/nbv/trigger_frustum_visualization', timeout=5)
        get_frustum = rospy.ServiceProxy(
                '/nbv/trigger_frustum_visualization', TriggerFrustumVisualization)
        get_frustum(camera_pose)
    except (rospy.ServiceException, rospy.exceptions.ROSException), e:
        rospy.logwarn(str(e))


def get_all_object_types_and_ids():
    try:
        rospy.loginfo("wait_for_service /env/world_model/get_all_objects_list")
        rospy.wait_for_service('/env/world_model/get_all_objects_list', timeout=5)
        get_all_objects_list = rospy.ServiceProxy(
                '/env/world_model/get_all_objects_list', GetAllObjectsList)
        return get_all_objects_list().allObjects
    except (rospy.ServiceException, rospy.exceptions.ROSException), e:
        rospy.logwarn(str(e))
        return 'aborted'

def get_object_types_and_ids_from_file():
    dbfilename = rospy.get_param('/rp_ism_node/dbfilename');
    pathSplit = dbfilename.split("/")
    dbName = pathSplit[-1]

    path = rospy.get_param('/scene_exploration_sm/IOPath')
    path = path.replace("XXX", dbName);
    rospy.loginfo("Parsed IntermediateObject path: " + str(path))

    tree = ET.parse(path)
    root = tree.getroot()
    object_types_and_ids = []
    for child in root:
        typeAndId = AsrTypeAndId()
        typeAndId.type = child.attrib["type"]
        typeAndId.identifier = child.attrib["identifier"]
        object_types_and_ids.append(typeAndId)
    return object_types_and_ids

def get_missing_object_types_and_ids():
    try:
        rospy.loginfo("wait_for_service /env/world_model/get_missing_object_list")
        rospy.wait_for_service('/env/world_model/get_missing_object_list', timeout=5)
        get_missing_objects = rospy.ServiceProxy(
            '/env/world_model/get_missing_object_list', GetMissingObjectList)
        return get_missing_objects().missingObjects
    except (rospy.ServiceException, rospy.exceptions.ROSException), e:
        rospy.logwarn(str(e))
        return 'aborted'


def get_search_object_types_and_ids():
    searched_object_types_and_ids = []
    if rospy.get_param("scene_exploration_sm/SearchAllobject"):
        all_object_types = get_all_object_types_and_ids()
        if (all_object_types == 'aborted'):
            return 'aborted'
        searched_object_types_and_ids = all_object_types
    else:
        searched_object_types_and_ids = get_object_types_and_ids_from_file()
    rospy.loginfo("Unfiltered object types and ids:\n" + str(searched_object_types_and_ids))

    missing_object_types_and_ids = get_missing_object_types_and_ids()
    if missing_object_types_and_ids == 'aborted':
        return 'aborted'
    filtered_object_types_and_ids = []
    for object_type_and_id in searched_object_types_and_ids:
        for missing_object_type_and_id in missing_object_types_and_ids:
            if object_type_and_id.type == missing_object_type_and_id.type and object_type_and_id.identifier == missing_object_type_and_id.identifier:
                missing_object_types_and_ids.remove(missing_object_type_and_id)
                filtered_object_types_and_ids.append(missing_object_type_and_id)
    rospy.loginfo("Searched object types and ids (filtered now):\n" + str(filtered_object_types_and_ids))
    return filtered_object_types_and_ids

