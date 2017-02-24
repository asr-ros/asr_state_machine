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
import json
import os
import time as t
import __builtin__
import move
import subprocess, os, signal
import shutil
import sys

from select import select
from world_model.srv import EmptyViewportList, EmptyCompletePatterns, GetAllObjectsList
from geometry_msgs.msg import (Pose, PoseWithCovariance, 
    PoseWithCovarianceStamped, Point, Quaternion, Twist)
from asr_msgs.msg import AsrViewport
from evaluation_decorators import *
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from recognizer_prediction_ism.srv import SetLogDir
from os.path import expanduser
from states.direct_search_states import execute_direct_search_action

"""
Contains all initialization states and useful methods for initialization.
"""
def reset_ism():
    """ Reset ism node counter"""
    rospy.loginfo("Resetting rp_ism_node.")
    try:
        rospy.loginfo("wait_for_service /rp_ism_node/reset")
        rospy.wait_for_service('/rp_ism_node/reset')
        reset = rospy.ServiceProxy('/rp_ism_node/reset', Empty)
        reset()
    except (rospy.ServiceException, rospy.exceptions.ROSException), e:
        rospy.logwarn(str(e))
        rospy.logwarn("Could not reset rp_ism_node")


def create_log_folder():
    rospy.loginfo("Creating log folder.")
    """
    Creates folder for log files and copies scene_file to this folder if on
    odete. Returns folder path
    """
    local_time = t.strftime("%Y-%m-%dT%H-%M-%S", t.localtime())
    home = expanduser("~")
    path = home + "/log/" + local_time 
    os.makedirs(path)
    open(path + '/log.csv', 'a').close()

    # copy scene file to log dir
    if True:
        scene_path = None
        try:
            rospy.loginfo("wait_for_service /env/world_model/get_all_objects_list")
            rospy.wait_for_service('/env/world_model/get_all_objects_list')
            get_all_objects_list = rospy.ServiceProxy(
                '/env/world_model/get_all_objects_list', GetAllObjectsList)
            scene_path = get_all_objects_list().scenePath
            
            rospy.loginfo("wait_for_service /rp_ism_node/set_log_dir")
            rospy.wait_for_service('/rp_ism_node/set_log_dir')
            set_log_dir = rospy.ServiceProxy(
		'rp_ism_node/set_log_dir', SetLogDir)
	    set_log_dir(path)
        except (rospy.ServiceException, rospy.exceptions.ROSException), e:
            rospy.logwarn(str(e))

        if scene_path:
           shutil.copy(scene_path,path)

    return path


def clear_world_model():
    rospy.loginfo("Clearing world_model.")
    """
    Delete all viewports and objects from world model
    """
    try:
        rospy.loginfo("wait_for_service /env/world_model/empty_found_object_list")
        rospy.wait_for_service('/env/world_model/empty_found_object_list')
        rospy.loginfo("wait_for_service /env/world_model/empty_viewport_list")
        rospy.wait_for_service('/env/world_model/empty_viewport_list')
        rospy.loginfo("wait_for_service /env/world_model/empty_complete_patterns")
        rospy.wait_for_service('/env/world_model/empty_complete_patterns')
        clear_object_list = rospy.ServiceProxy('/env/world_model/empty_found_object_list',
                                            Empty)
        clear_viewport_list = rospy.ServiceProxy('/env/world_model/empty_viewport_list',
                                            EmptyViewportList)
        empty_complete_patterns = rospy.ServiceProxy('/env/world_model/empty_complete_patterns',
                                            EmptyCompletePatterns)
        clear_object_list()
        clear_viewport_list('all')
        empty_complete_patterns()
    except (rospy.ServiceException, rospy.exceptions.ROSException) as e:
        rospy.logwarn(e)
        rospy.logwarn("Could not clear world model")


def clear_costmap():
    rospy.loginfo("Clearing costmap.")
    """
    Clears costmap from move base
    """
    try:
        rospy.loginfo("wait_for_service /move_base/clear_costmaps")
        rospy.wait_for_service('/move_base/clear_costmaps')
        clear_costmaps = rospy.ServiceProxy('/move_base/clear_costmaps',
                                            Empty)
        clear_costmaps()
    except (rospy.ServiceException, rospy.exceptions.ROSException) as e:
        rospy.loginfo("Could not clear costmaps")

def get_initial_objects():
    initial_objects = []
    rospy.loginfo("Object types to look for in the first step")
    rospy.loginfo("Enter nothing if you want to look for all objects in the scene")
    rospy.loginfo("They will only be used in indirect_search")
    timeout = 0.1
    while not rospy.is_shutdown():
        # Wait for input while check if node is_shutdown
        obj, _, _ = select([sys.stdin], [], [], timeout)
        if obj:
            currentLine = str(sys.stdin.readline().strip())
            if currentLine is not "":
                initial_objects.append(currentLine)
            else:
                break

    return initial_objects


def start_rosbag(path):
    rospy.loginfo("Starting rosbag.")
    """
    Starts rosbag in the specified folder with the topics.
    """
    
    #TODO move topics to settings file
    rospy.loginfo("wait_for_service /rp_ism_node/set_log_dir")
    rospy.wait_for_service('/rp_ism_node/set_log_dir')
    posePredictionMarkersPublisherName = rospy.get_param("/rp_ism_node/posePredictionMarkersPublisherName")
    sceneMarkersPublisherName = rospy.get_param("/rp_ism_node/sceneMarkersPosePredictionPublisherName")
    visualization_topic = rospy.get_param("/rp_ism_node/sceneMarkersSceneRecognitionPublisherName")

    waypoints = rospy.get_param("/scene_exploration_sm/waypoints")

    stereo_visualization_marker = '/stereo/visualization_marker'
    stereo_objects = '/stereo/objects'

    tf = rospy.get_param("/scene_exploration_sm/tf")
    tf_static = rospy.get_param("/scene_exploration_sm/tf_static")
    map = rospy.get_param("/scene_exploration_sm/map")
    move_base_footprint_stamped = rospy.get_param("/scene_exploration_sm/move_base_footprint_stamped")
    odom = rospy.get_param("/scene_exploration_sm/odom")
    move_base_plan = rospy.get_param("/scene_exploration_sm/move_base_plan")
    move_base_local_plan = rospy.get_param("/scene_exploration_sm/move_base_local_plan")
    move_base_goal = rospy.get_param("/scene_exploration_sm/move_base_goal")
    move_base_global_costmap = rospy.get_param("/scene_exploration_sm/move_base_global_costmap")
    move_base_local_costmap = rospy.get_param("/scene_exploration_sm/move_base_local_costmap")
    particlecloud = rospy.get_param("/scene_exploration_sm/particlecloud")

    rospy.loginfo("wait_for_service /nbv/trigger_old_frustum_visualization")
    rospy.wait_for_service('/nbv/trigger_old_frustum_visualization')

    nbv_frustum_array = rospy.get_param("/nbv/frustumVisualization")
    nbv_frustum_object = rospy.get_param("/nbv/frustumObjectsVisualization")
    nbv_object_meshes = rospy.get_param("/nbv/objectsVisualization")
    nbv_iteration_viz = rospy.get_param("/nbv/iterationVisualization")
    nbv_object_normals = rospy.get_param("/nbv/objectNormalsVisualization")
    nbv_cropbox = rospy.get_param("/nbv/cropBoxVisualization")
    nbv_IK = rospy.get_param("/nbv/IKVisualization")

    rospy.loginfo("wait_for_service /env/world_model/empty_found_object_list")
    rospy.wait_for_service('/env/world_model/empty_found_object_list')
    world_model_found = '/env/world_model/found_object_visualization'
    constellation_fake = '/fake_object_recognition/constellation_visualization'

    container_status = rospy.get_param("/scene_exploration_sm/container_status")
    container_init = rospy.get_param("/scene_exploration_sm/container_init")
    container_structure = rospy.get_param("/scene_exploration_sm/container_structure")

    threeD_models_topic = rospy.get_param("/scene_exploration_sm/threeD_models_topic")


    topics=[posePredictionMarkersPublisherName, sceneMarkersPublisherName, visualization_topic, waypoints,
        stereo_visualization_marker, stereo_objects, tf, tf_static, map,move_base_footprint_stamped, odom,
        move_base_plan, move_base_local_plan, move_base_goal, move_base_global_costmap, move_base_local_costmap, particlecloud,
        nbv_frustum_array, nbv_frustum_object, nbv_object_meshes, nbv_iteration_viz, nbv_object_normals, nbv_cropbox, nbv_IK,
            world_model_found, constellation_fake, container_status, container_init, container_structure, threeD_models_topic]

    command = "rosbag record -b 1000 -O OBJ${1}"
    for topic in topics:
        command +=  " " + topic
    print command
    rosbag_proc = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, cwd=path)
    return rosbag_proc


class ObjectSearchInit(smach.State):

    def __init__(self):
        smach.State.__init__(self,
                outcomes=['succeeded', 'aborted'])

    @log
    @timed
    def execute(self, userdata):
        rospy.loginfo('Executing OBJECT_SEARCH_INIT')

        clear_costmap()
        clear_world_model()
        reset_ism()
        result = execute_direct_search_action("Reset")
        if (result == 'aborted'):
            return 'aborted'

        return 'succeeded'


class SearchInit(smach.State):
    """
    Initial state for object search.
    """

    def __init__(self, searched_object_types=None):
        smach.State.__init__(self,
                outcomes=['succeeded', 'aborted'],
                output_keys=['searched_object_types'])
        self.searched_object_types = searched_object_types

    @log
    @timed
    def execute(self, userdata):
        rospy.loginfo('Executing SEARCH_INIT')

        clear_costmap()
        clear_world_model()
        reset_ism()
        first_object_types_to_search = []
        if not self.searched_object_types:
            try:
                rospy.loginfo("wait_for_service /env/world_model/get_all_objects_list")
                rospy.wait_for_service('/env/world_model/get_all_objects_list')
                get_all_objects_list = rospy.ServiceProxy(
                    '/env/world_model/get_all_objects_list', GetAllObjectsList)
                for object_type_and_id in get_all_objects_list().allObjects:
                    first_object_types_to_search.append(object_type_and_id.type)
            except (rospy.ServiceException, rospy.exceptions.ROSException), e:
                rospy.logwarn(str(e))
                return 'aborted'
        else:
            first_object_types_to_search = self.searched_object_types

        userdata.searched_object_types = first_object_types_to_search

        return 'succeeded'

