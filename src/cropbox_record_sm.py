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

from common_sm import GetMoveRobotSateMachine
from states.init import SearchInit
from states.move import PTUPoseCorrection
from states.nbv import NBVSetPointCloud, NextBestView, NextBestViewUpdate
from states.cropbox_search_states import CropBoxGeneration
from states.record_states import CropboxStateRecording
from states.visualize_waypoints import VisualizeWaypoints

class CropBoxRecordStateMachine():

    sm_init = smach.StateMachine(outcomes=['succeeded',
                                           'aborted'])
    with sm_init:
        smach.StateMachine.add('SEARCH_INIT',
                               SearchInit(),
                               transitions={'succeeded':'CROPBOX_GENERATION',
                                            'aborted':'aborted'},
                               remapping={'searched_object_types':'searched_object_types'})

        smach.StateMachine.add('CROPBOX_GENERATION',
                               CropBoxGeneration(),
                               transitions={'succeeded':'NBV_SET_POINT_CLOUD',
                                            'aborted':'aborted'},
                               remapping={'object_pointcloud':'object_pointcloud'})

        smach.StateMachine.add('NBV_SET_POINT_CLOUD',
                               NBVSetPointCloud(),
                               transitions={'succeeded':'succeeded',
                                            'aborted':'aborted',
                                            'too_many_deactivated_normals':'aborted'},
                               remapping={'object_pointcloud':'object_pointcloud'})



    sm_cropbox_record = smach.StateMachine(outcomes=['aborted',
                                                     'finished'])

    with sm_cropbox_record:
        smach.StateMachine.add('CROPBOX_RECORD_INIT',
                               sm_init,
                               transitions={'succeeded':'NBV_CALCULATION',
                                            'aborted':'aborted'})

        smach.StateMachine.add('NBV_CALCULATION',
                               NextBestView(),
                               transitions={'found_next_best_view':'MOVE_ROBOT_TO_VIEW',
                                            'aborted':'aborted',
                                            'no_nbv_found':'finished',
                                            'nbv_update_point_cloud':'MOVE_ROBOT_TO_VIEW'},
                               remapping={'goal_camera_pose':'goal_camera_pose',
                                          'goal_robot_pose':'goal_robot_pose',
                                          'goal_ptu_position':'goal_ptu_position',
                                          'searched_object_types':'searched_object_types'})

        smach.StateMachine.add('MOVE_ROBOT_TO_VIEW',
                               GetMoveRobotSateMachine(),
                               transitions={'succeeded':'PTU_POSE_CORRECTION',
                                            'aborted':'aborted'},
                               remapping={'goal_robot_pose':'goal_robot_pose',
                                          'goal_ptu_position':'goal_ptu_position'})

        smach.StateMachine.add('PTU_POSE_CORRECTION',
                               PTUPoseCorrection(),
                               transitions={'succeeded':'VISUALIZE_WAYPOINT',
                                            'aborted':'VISUALIZE_WAYPOINT'},
                               remapping={'goal_camera_pose':'goal_camera_pose'})

        smach.StateMachine.add('VISUALIZE_WAYPOINT',
                               VisualizeWaypoints(),
                               transitions={'succeeded':'NBV_UPDATE_POINT_CLOUD'})

        smach.StateMachine.add('NBV_UPDATE_POINT_CLOUD',
                               NextBestViewUpdate(),
                               transitions={'succeeded':'STATE_RECORDING',
                                            'aborted':'aborted',
                                            'no_nbv_found':'finished'},
                               remapping={'goal_camera_pose':'goal_camera_pose',
                                          'searched_object_types':'searched_object_types',
                                          'deactivated_object_normals_count':'deactivated_object_normals_count'})

        smach.StateMachine.add('STATE_RECORDING',
                               CropboxStateRecording(),
                               transitions={'succeeded':'NBV_CALCULATION',
                                            'aborted':'aborted'},
                               remapping={'goal_camera_pose':'goal_camera_pose',
                                         'goal_robot_pose':'goal_robot_pose',
                                         'goal_ptu_position':'goal_ptu_position',
                                         'deactivated_object_normals_count':'deactivated_object_normals_count'})


