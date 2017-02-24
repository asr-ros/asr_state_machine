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

from states.nbv import NextBestView, NextBestViewUpdate
from states.move import PTUPoseCorrection, MovePTU, MoveBase, FakeMoveBase
from states.object_detection import ObjectDetection
from states.visualize_waypoints import VisualizeWaypoints

def GetMoveRobotSateMachine():
    sm_move = smach.Concurrence(outcomes=['succeeded',
                                          'aborted'],
                                default_outcome='aborted',
                                outcome_map={'succeeded':{'MOVE_BASE':'succeeded',
                                                          'MOVE_PTU':'succeeded'}},
                                input_keys=['goal_robot_pose',
                                            'goal_ptu_position'])

    with sm_move:
        smach.Concurrence.add('MOVE_PTU',
                              MovePTU(),
                              remapping={'goal_ptu_position':'goal_ptu_position'})

        if rospy.get_param("/scene_exploration_sm/UseFakeMoveBase") is True:
            smach.Concurrence.add('MOVE_BASE',
                                  FakeMoveBase(),
                                  remapping={'goal_robot_pose':'goal_robot_pose'})
        else:
            smach.Concurrence.add('MOVE_BASE',
                                  MoveBase(),
                                  remapping={'goal_robot_pose':'goal_robot_pose'})

    return sm_move


def GetRelationBasedSearchStateMachine():

    sm_relation_based_search = smach.StateMachine(outcomes=['found_objects',
                                                            'aborted',
                                                            'no_nbv_found'])
    with sm_relation_based_search:

        smach.StateMachine.add('NBV_CALCULATION',
                               NextBestView(),
                               transitions={'found_next_best_view':'MOVE_ROBOT_TO_VIEW',
                                            'aborted':'aborted',
                                            'no_nbv_found':'no_nbv_found',
                                            'nbv_update_point_cloud':'NBV_UPDATE_POINT_CLOUD'},
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
                               transitions={'succeeded':'OBJECT_DETECTION'})

        smach.StateMachine.add('OBJECT_DETECTION',
                               ObjectDetection(),
                               transitions={'no_objects_found':'NBV_UPDATE_POINT_CLOUD',
                                            'found_objects':'found_objects',
                                            'aborted':'aborted'},
                               remapping={'searched_object_types':'searched_object_types',
                                          'detected_objects':'detected_objects'})

        smach.StateMachine.add('NBV_UPDATE_POINT_CLOUD',
                               NextBestViewUpdate(),
                               transitions={'succeeded':'NBV_CALCULATION',
                                            'aborted':'aborted',
                                            'no_nbv_found':'no_nbv_found'},
                               remapping={'goal_camera_pose':'goal_camera_pose',
                                          'searched_object_types':'searched_object_types'})

    return sm_relation_based_search;

