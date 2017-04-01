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
import tf
import math
import pickle
import os

from asr_next_best_view.srv import *
from asr_next_best_view.msg import RobotStateMessage, NormalsInfo
from asr_world_model.srv import GetViewportList, FilterViewportDependingOnAlreadyVisitedViewports
from asr_msgs.msg import (AsrAttributedPointCloud, AsrAttributedPoint, AsrViewport)
from geometry_msgs.msg import (Pose, PoseWithCovariance,
                               PoseWithCovarianceStamped, Point, Quaternion, Twist)

from common.evaluation_decorators import *
import __builtin__
import common.state_acquisition

"""
States and methods for using the Next-Best-View package.
"""

global rejectedPointCloud
rejectedPointCloud = []
global numSetPointCloud
numSetPointCloud = 0
global useGoalCameraPoseAtNextNBVUpdatePointCloud
useGoalCameraPoseAtNextNBVUpdatePointCloud = False

class NBVSetPointCloud(smach.State):
    """
    Initializes the point cloud for next_best_view.
    If there is already a point cloud set, this state overwrites it.
    """

    def __init__(self):
        smach.State.__init__(
            self, outcomes=['succeeded', 'aborted', 'too_many_deactivated_normals'],
            input_keys=['object_pointcloud'])
        self.isInitDone = 0

    def initNode(self):
        rospy.loginfo('Init NBV_SET_POINT_CLOUD')
        self.isInitDone = 1
        self.percentage_too_many_deactivated = rospy.get_param('/scene_exploration_sm/percentageTooManyDeactivatedNormals')
        rospy.loginfo("percentageTooManyDeactivatedNormals: " + str(self.percentage_too_many_deactivated))
        if rospy.get_param("~SavePointClouds"):
            os.makedirs(__builtin__.log_dir + "/PointCloud")

    @log
    @timed
    def execute(self, userdata):
        rospy.loginfo('Executing NBV_SET_POINT_CLOUD')
        if self.isInitDone == 0:
            self.initNode()

        try:
            rospy.wait_for_service('/nbv/set_point_cloud', timeout=5)
            rospy.wait_for_service('/nbv/remove_objects', timeout=5)
            rospy.wait_for_service('/env/asr_world_model/get_viewport_list', timeout=5)
        except rospy.exceptions.ROSException, e:
            rospy.loginfo("Couldn't reach service")
            return 'aborted'
        try:
            setRobotState()

            # services
            set_pc = rospy.ServiceProxy('/nbv/set_point_cloud', SetAttributedPointCloud)
            invalidate_point_cloud = rospy.ServiceProxy('/nbv/remove_objects', RemoveObjects)
            get_viewports = rospy.ServiceProxy('/env/asr_world_model/get_viewport_list', GetViewportList)

            # used to get points in nbv service call
            pointCloud = userdata.object_pointcloud
            # append previously removed hypothesis
            if rospy.get_param("~recoverLostHypothesis"):
                global rejectedPointCloud
                pointCloud.elements += rejectedPointCloud
                if (len(rejectedPointCloud) > 0):
                    rospy.loginfo("added " + str(len(rejectedPointCloud)) + " old points to pc")
                rejectedPointCloud = []
            viewports = get_viewports('all').viewport_list
            set_pc_rsp = set_pc(pointCloud, viewports)
            
            # save pointcloud to debug later
            if rospy.get_param("~SavePointClouds"):
                global numSetPointCloud
                pointCloudFile = open(__builtin__.log_dir + "/PointCloud/PointCloud_" + str(numSetPointCloud), "wb")
                viewportsFile = open(__builtin__.log_dir + "/PointCloud/Viewports_" + str(numSetPointCloud), "wb")
                pickle.dump(pointCloud, pointCloudFile)
                pickle.dump(viewports, viewportsFile)
                pointCloudFile.close()
                viewportsFile.close()
                numSetPointCloud += 1

            # print if too many object hypothesis were removed from earlier viewports per object
            # check if all hypothesis are covered by earlier viewports
            all_percentages_too_low = True

            for normalInfo in set_pc_rsp.normals_per_object:
                rospy.loginfo("Number of active object normals in original point cloud for type " + \
                    normalInfo.type + " and id " + normalInfo.identifier + ": " + str(normalInfo.active_normals))
                rospy.loginfo("Number of deactivated normals for type " + \
                    normalInfo.type + " and id " + normalInfo.identifier + ": " + str(normalInfo.deactivated_object_normals))

                if normalInfo.active_normals == 0:
                    quotient = 0.0
                else:
                    quotient = 1.0 * normalInfo.deactivated_object_normals / normalInfo.active_normals
                rospy.loginfo("Quotient of deactivated_object_normals / set_pc_rsp.active_normals for type " + \
                    normalInfo.type + " and id " + normalInfo.identifier + ": " + str(quotient))
                if quotient < self.percentage_too_many_deactivated:
                    all_percentages_too_low = False
                    break

            # whole pc will be removed because earlier viewports cover most hypothesis
            if all_percentages_too_low == True:
                rospy.logwarn("Percentage of active normals in cloud too low for all types and ids. " + \
                    "Calculating different point cloud for NBV estimation instead.")
                if rospy.get_param("~recoverLostHypothesis"):
                    get_pc = rospy.ServiceProxy("/nbv/get_point_cloud", GetAttributedPointCloud)
                    rejectedPointCloud += get_pc().point_cloud.elements
                return 'too_many_deactivated_normals'
            else:
                # only some or none hypothesis are covered by earlier hypothesis
                if rospy.get_param("~recoverLostHypothesis"):
                    get_pc = rospy.ServiceProxy("/nbv/get_point_cloud", GetAttributedPointCloud)
                    filteredPC = get_pc()
                for normalInfo in set_pc_rsp.normals_per_object:
                    if normalInfo.active_normals == 0:
                        quotient = 0.0
                    else:
                        quotient = 1.0 * normalInfo.deactivated_object_normals / normalInfo.active_normals
                    if quotient >= self.percentage_too_many_deactivated :
                        # objects will be removed
                        rospy.logwarn("Percentage of active normals in cloud too low for type " + \
                            normalInfo.type + " and id " + normalInfo.identifier + ". Erase all normals for type and id in point_cloud.")

                        # save objects that will be removed
                        if rospy.get_param("~recoverLostHypothesis"):
                            # recover hypothesis that we invalidate later
                            for obj in filteredPC.point_cloud.elements:
                                if (obj.type == normalInfo.type and obj.identifier == normalInfo.identifier):
                                    rejectedPointCloud.append(obj)
                        # remove objects from pc
                        result = invalidate_point_cloud(normalInfo.type, normalInfo.identifier)
                        if not result.is_valid:
                            # PC is empty
                            return 'too_many_deactivated_normals'

            if set_pc_rsp.is_valid == False:
                return 'aborted'
            else:
                return 'succeeded'

        except rospy.ServiceException, e:
            rospy.logwarn(str(e))
            return 'aborted'


class NextBestView(smach.State):
    """
    Calculates NextBestView given a pose
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['found_next_best_view', 'aborted', 'no_nbv_found', 'nbv_update_point_cloud'],
            output_keys=['goal_camera_pose', 'goal_robot_pose', 'goal_ptu_position', 'searched_object_types'])

    @log
    @timed
    def execute(self, userdata):
        rospy.loginfo('Executing NEXT_BEST_VIEW')

        rospy.wait_for_service('/nbv/next_best_view')
        try:
            setRobotState()

            get_nbv = rospy.ServiceProxy('/nbv/next_best_view', GetNextBestView)
            min_utility_for_moving = rospy.get_param("/scene_exploration_sm/min_utility_for_moving")

            # get_camera_pose_cpp() gets actual pose of the camera of the robot.
            current_camera_pose = state_acquisition.get_camera_pose_cpp()
            rospy.loginfo("This is the current camera pose, given to NBV: " + str(current_camera_pose))

            get_nbv_rsp = get_nbv(current_camera_pose)

            #If no nbv found at all, get next estimation from pose prediction
            if get_nbv_rsp.found == False:
                rospy.logwarn("Could not find any pose with rating > 0 in next best view estimation, aborting...")
                if rospy.get_param("~recoverLostHypothesis"):
                    get_pc = rospy.ServiceProxy("/nbv/get_point_cloud", GetAttributedPointCloud)
                    global rejectedPointCloud
                    rejectedPointCloud += get_pc().point_cloud.elements
                return 'no_nbv_found'

            rospy.loginfo("This is the camera pose, returned by NBV: " + str(get_nbv_rsp))

            # This is the camera pose calculated by the nbv but most certainly
            # it won't be reached due to uncertainties
            userdata.goal_camera_pose = get_nbv_rsp.resulting_pose

            # objects to search in this nbv pose
            userdata.searched_object_types = get_nbv_rsp.object_type_name_list

            # construct goal for moving base and ptu from nbv result
            move_pose = Pose()
            quaternion = tf.transformations.quaternion_from_euler(0, 0, get_nbv_rsp.robot_state.rotation)
            move_pose.position = Point(get_nbv_rsp.robot_state.x,
                                       get_nbv_rsp.robot_state.y,
                                       0)
            move_pose.orientation = Quaternion(*quaternion)

            userdata.goal_robot_pose = move_pose

            # rad to degree
            pan = math.trunc(get_nbv_rsp.robot_state.pan * (180 / numpy.pi))
            tilt = math.trunc(get_nbv_rsp.robot_state.tilt * (180 / numpy.pi))
            rospy.logdebug("PTU from nbv robot state in deg - pan: " + str(pan) + "| tilt: " + str(tilt))
            userdata.goal_ptu_position = [pan, tilt]

            viewport = AsrViewport()
            viewport.pose = get_nbv_rsp.resulting_pose
            viewport.object_type_name_list = get_nbv_rsp.object_type_name_list

            if rospy.get_param("/scene_exploration_sm/filter_viewports"):
                rospy.wait_for_service('/env/asr_world_model/filter_viewport_depending_on_already_visited_viewports')
                filterViewportServ = rospy.ServiceProxy('/env/asr_world_model/filter_viewport_depending_on_already_visited_viewports',
                                        FilterViewportDependingOnAlreadyVisitedViewports)
                filterViewportResp = filterViewportServ(viewport)

                if filterViewportResp.isBeenFiltered:
                    rospy.logwarn("Viewport was already visited -> filtered objectTypes to search: " + str(filterViewportResp.filteredViewport.object_type_name_list))
                    if len(filterViewportResp.filteredViewport.object_type_name_list) == 0:
                        rospy.logwarn("No objects to search -> nbv_update_point_cloud")
                        global useGoalCameraPoseAtNextNBVUpdatePointCloud
                        useGoalCameraPoseAtNextNBVUpdatePointCloud = True
                        #the normals deleted in nbv_update_point_cloud get lost and are not in rejectedPointCloud
                        return 'nbv_update_point_cloud'
                    else:
                        viewport = filterViewportResp.filteredViewport
                        userdata.searched_object_types = filterViewportResp.filteredViewport.object_type_name_list


            if (min_utility_for_moving > get_nbv_rsp.utility):
                rospy.logwarn("Utility in found next best view estimate too low for moving to it, aborting...")
                if rospy.get_param("~recoverLostHypothesis"):
                    get_pc = rospy.ServiceProxy("/nbv/get_point_cloud", GetAttributedPointCloud)
                    global rejectedPointCloud
                    rejectedPointCloud += get_pc().point_cloud.elements
                return 'no_nbv_found'

            # trigger nbv visualization
            trigger_nbv_vis = rospy.ServiceProxy('/nbv/trigger_frustums_and_point_cloud_visualization', TriggerFrustumsAndPointCloudVisualization)
            trigger_nbv_vis(viewport)

            # write nbv results into log folder if log_dir is specified
            if hasattr(__builtin__, 'log_dir'):
                with open(__builtin__.log_dir + '/nbv_results.csv', 'a') as log:
                    logwriter = csv.writer(log, delimiter=' ',
                                           quotechar='|', quoting=csv.QUOTE_MINIMAL)
                    logwriter.writerow([time(), str(get_nbv_rsp)])

            return 'found_next_best_view'
        except rospy.ServiceException, e:
            rospy.logwarn("ServiceException from NextBestView: " + str(e))
            rospy.logwarn(e)
            return 'aborted'


class NextBestViewUpdate(smach.State):
    """
    Updates the pointcloud for next_best_view with the actual pose.

    Note that the actual pose differs from the calculated nbv pose, so
    that we have to get the actual camera pose to update here.
    """

    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'no_nbv_found'],
            input_keys=['goal_camera_pose', 'searched_object_types'],
            output_keys=['deactivated_object_normals_count'])
        self.searched_object_types = None
        self.isInitDone = 0

    def initNode(self):
        rospy.loginfo('Init NEXT_BEST_VIEW_UPDATE')
        self.isInitDone = 1
        self.mode = rospy.get_param("/scene_exploration_sm/mode")
        self.resetRemainingAttempts()
        
    def resetRemainingAttempts(self):
        # In cropbox record we allow more remaining_attempts, because we have only one PointCloud to set
        # It is possible that there will be a cycle of views which NBV trys to reach but not successfully
        if self.mode == 6:
            self.remaining_attempts = 10
        else:
            #Required for checking if we drive around hypotheses without deactivating normals.
            #We only accept to do this twice. Otherwise we need new predicted poses.
            self.remaining_attempts = 1
        #Required for checking if search for objects at the same view.
        self.last_calculated_pose = None
        rospy.loginfo("Reset remaining attempts to: " + str(self.remaining_attempts))


    @log
    @timed
    @key_press
    def execute(self, userdata):
        rospy.loginfo('Executing NEXT_BEST_VIEW_UPDATE')
        if self.isInitDone == 0:
            self.initNode()

        rospy.wait_for_service('/nbv/update_point_cloud')
        setRobotState()

        # In Cropbox search/record we have to update the normals for the placeholder object, because the point_cloud consists only of it
        if self.mode == 5 or self.mode == 6:
            self.searched_object_types = [rospy.get_param('/scene_exploration_sm/PlaceholderObjectTypeInCropboxSearch')]
        else:
            #check if searched_object_types is only one object and transform to list
            if not isinstance(userdata.searched_object_types, list):
            	self.searched_object_types = [userdata.searched_object_types]
            else:
            	self.searched_object_types = userdata.searched_object_types

        try:
            nbv_update = rospy.ServiceProxy('/nbv/update_point_cloud', UpdatePointCloud)

            # We get stuck in a loop here, when searching for objects at hypotheses, that are erroneous.
            # Problem 1: Our navigation does not exactly reach a nbv. It tries to reach it without success over and over.
            # Problem 2: NBV calculation produces views in which no normals can be deleted and the robot repeatedly tries out views around existing hypotheses.
            global useGoalCameraPoseAtNextNBVUpdatePointCloud
            if useGoalCameraPoseAtNextNBVUpdatePointCloud:
                useGoalCameraPoseAtNextNBVUpdatePointCloud = False
                current_camera_pose = userdata.goal_camera_pose
            else:
                current_camera_pose = state_acquisition.get_camera_pose()

            rospy.loginfo("Deactivating normals in predicted cloud based on camera pose : " + str(current_camera_pose) + " and on objects " + str(list(self.searched_object_types)))
            rsp = nbv_update(current_camera_pose, self.searched_object_types)
            rospy.loginfo("Number of deactivated normals: " + str(rsp.deactivated_object_normals))
            userdata.deactivated_object_normals_count = rsp.deactivated_object_normals

            #We need to update differently since deactivating no normals risks to end up in infinite loop.
            if rsp.deactivated_object_normals == 0:
                rospy.logwarn("View, reached by robot, differs from NBV and allows no update.")
                #In order not to update from the same pose over and over again, we remember the old pose and check for approximate equality.
                if state_acquisition.approx_equal_pose(userdata.goal_camera_pose, self.last_calculated_pose, 0.2):
                    rospy.loginfo("Robot did not move enough after NBV estimation. Updating cloud with NBV estimate: " + str(userdata.goal_camera_pose))
                    #We only delete those normals in the nbv to have the chance to search for objects at other places with the same pose predictions.
                    rsp = nbv_update(userdata.goal_camera_pose, self.searched_object_types)
                    rospy.loginfo("Number of deactivated normals: " + str(rsp.deactivated_object_normals))
                    self.resetRemainingAttempts()
                else:
                    #If remaining_attempts == 1: We still have a chance to be successful in deleting normals in the next view.
                    self.remaining_attempts -= 1
                    self.last_calculated_pose = userdata.goal_camera_pose
                    rospy.loginfo("Remaining attempts till no_nbv_found: " + str(self.remaining_attempts))

                if self.remaining_attempts <= 0:
                    #Driving around hypotheses is a good sign for having no good data in the given pose predictions.
                    rospy.loginfo("Two NBV estimations deactivated no normals. Aborting object search loop.")
                    self.resetRemainingAttempts()
                    if rospy.get_param("~recoverLostHypothesis"):
                        get_pc = rospy.ServiceProxy("/nbv/get_point_cloud", GetAttributedPointCloud)
                        global rejectedPointCloud
                        rejectedPointCloud += get_pc().point_cloud.elements
                    return 'no_nbv_found';
            else :
                self.resetRemainingAttempts()
            return 'succeeded'
        except rospy.ServiceException, e:
            rospy.logwarn(str(e))
            return 'aborted'


def setRobotState():
    # try to get current robot config and set it in the nbv
    getRobotState = state_acquisition.GetRobotState()
    robotState = getRobotState.get_robot_state()
    rospy.loginfo("Transmitting current robot state to NBV: (Pan: " + str(robotState.pan) + ", Tilt: " + str(robotState.tilt) + ", Rotation: " + str(robotState.rotation) + ", X: " + str(robotState.x) + ", Y: " + str(robotState.y) + ")")

    rospy.wait_for_service('/nbv/set_init_robot_state')
    set_init_state = rospy.ServiceProxy('/nbv/set_init_robot_state', SetInitRobotState)
    set_init_state(robotState)
