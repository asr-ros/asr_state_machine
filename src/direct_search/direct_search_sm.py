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

from common.common_sm import GetMoveRobotSateMachine
from direct_search_states import DirectSearchInit, GetGoalCameraPose, CheckSearchFinished, FrustumViz
from common.object_detection import ObjectDetection
from common.move import MovePTU, PTUPoseCorrection
from indirect_search.ism import SceneRecognition
from common.visualize_waypoints import VisualizeWaypoints

class DirectSearchStateMachine():

    def getSmSearch():
        sm_search = smach.StateMachine(outcomes=['found_objects',
                                                 'aborted',
                                                 'nowhere_left_to_search'],
                                       input_keys=['searched_object_types_and_ids'])

        with sm_search:

            smach.StateMachine.add('GET_GOAL_CAMERA_POSE',
                                   GetGoalCameraPose(),
                                   transitions={'MoveRobot':'MOVE_ROBOT',
                                                'MovePTUOnly':'MOVE_PTU_ONLY',
                                                'nowhere_left_to_search':'nowhere_left_to_search',
                                                'aborted':'aborted'},
                                   remapping={'goal_camera_pose':'goal_camera_pose',
                                              'goal_robot_pose':'goal_robot_pose',
                                              'goal_ptu_position':'goal_ptu_position',
                                              'searched_object_types_and_ids':'searched_object_types_and_ids',
                                              'filtered_searched_object_types':'filtered_searched_object_types'})

            smach.StateMachine.add('MOVE_ROBOT',
                                   GetMoveRobotSateMachine(),
                                   transitions={'succeeded':'PTU_POSE_CORRECTION',
                                                'aborted':'aborted'},
                                   remapping={'goal_robot_pose':'goal_robot_pose',
                                              'goal_ptu_position':'goal_ptu_position'})

            smach.StateMachine.add('MOVE_PTU_ONLY',
                                   MovePTU(),
                                   transitions={'succeeded':'PTU_POSE_CORRECTION',
                                                'aborted':'aborted'},
                                   remapping={'goal_ptu_position':'goal_ptu_position'})

            smach.StateMachine.add('PTU_POSE_CORRECTION',
                                   PTUPoseCorrection(),
                                   transitions={'succeeded':'VISUALIZE_FRUSTUM',
                                                'aborted':'VISUALIZE_FRUSTUM'},
                                   remapping={'goal_camera_pose':'goal_camera_pose'})

            smach.StateMachine.add('VISUALIZE_FRUSTUM',
                                   FrustumViz(),
                                   transitions={'succeeded':'VISUALIZE_WAYPOINT'})

            smach.StateMachine.add('VISUALIZE_WAYPOINT',
                                   VisualizeWaypoints(),
                                   transitions={'succeeded':'OBJECT_DETECTION'})

            smach.StateMachine.add('OBJECT_DETECTION',
                                   ObjectDetection(),
                                   transitions={'found_objects':'found_objects',
                                                'no_objects_found':'GET_GOAL_CAMERA_POSE',
                                                'aborted':'aborted'},
                                   remapping={'searched_object_types':'filtered_searched_object_types',
                                              'detected_objects':'detected_objects'})
        return sm_search



    sm_direct_search_as_standalone = smach.StateMachine(outcomes=['search_finished',
                                                     'aborted',
                                                     'found_all_objects',
                                                     'found_all_required_scenes',
                                                     'nowhere_left_to_search'])

    with sm_direct_search_as_standalone:
        smach.StateMachine.add('DIRECT_SEARCH_INIT',
                               DirectSearchInit(),
                               transitions={'object_poses_from_demonstrations_given':'OBJECT_LOCATIONS_BASED_SEARCH',
                                            'no_object_poses_from_demonstrations':'SCENE_EXPLORATION',
                                            'aborted':'aborted'},
                               remapping={'searched_object_types_and_ids':'searched_object_types_and_ids'})

        smach.StateMachine.add('OBJECT_LOCATIONS_BASED_SEARCH',
                               getSmSearch(),
                               transitions={'found_objects':'CHECK_SEARCH_FINISHED',
                                            'aborted':'aborted',
                                            'nowhere_left_to_search':'SCENE_EXPLORATION'},
                               remapping={'searched_object_types_and_ids':'searched_object_types_and_ids'})

        smach.StateMachine.add('SCENE_EXPLORATION',
                               getSmSearch(),
                               transitions={'found_objects':'CHECK_SEARCH_FINISHED',
                                            'aborted':'aborted',
                                            'nowhere_left_to_search':'nowhere_left_to_search'},
                               remapping={'searched_object_types_and_ids':'searched_object_types_and_ids'})


        smach.StateMachine.add('CHECK_SEARCH_FINISHED',
                               CheckSearchFinished(),
                               transitions={'search_finished':'search_finished',
                                            'search_not_finished':'SCENE_RECOGNITION',
                                            'aborted':'aborted'},
                               remapping={'searched_object_types_and_ids':'searched_object_types_and_ids'})

        smach.StateMachine.add('SCENE_RECOGNITION',
                               SceneRecognition(),
                               transitions={'found_scenes':'OBJECT_LOCATIONS_BASED_SEARCH',
                                            'found_all_required_scenes':'found_all_required_scenes',
                                            'found_all_objects':'found_all_objects',
                                            'aborted':'aborted'})



    sm_direct_search_for_object_search = smach.StateMachine(outcomes=['found_objects',
                                                                      'aborted',
                                                                      'nowhere_left_to_search'])

    with sm_direct_search_for_object_search:
        smach.StateMachine.add('DIRECT_SEARCH_INIT',
                               DirectSearchInit(),
                               transitions={'object_poses_from_demonstrations_given':'OBJECT_LOCATIONS_BASED_SEARCH',
                                            'no_object_poses_from_demonstrations':'SCENE_EXPLORATION',
                                            'aborted':'aborted'},
                               remapping={'searched_object_types_and_ids':'searched_object_types_and_ids'})

        smach.StateMachine.add('OBJECT_LOCATIONS_BASED_SEARCH',
                               getSmSearch(),
                               transitions={'found_objects':'found_objects',
                                            'aborted':'aborted',
                                            'nowhere_left_to_search':'SCENE_EXPLORATION'},
                               remapping={'searched_object_types_and_ids':'searched_object_types_and_ids'})

        smach.StateMachine.add('SCENE_EXPLORATION',
                               getSmSearch(),
                               transitions={'found_objects':'found_objects',
                                            'aborted':'aborted',
                                            'nowhere_left_to_search':'nowhere_left_to_search'},
                               remapping={'searched_object_types_and_ids':'searched_object_types_and_ids'})
