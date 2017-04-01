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
from direct_search_states import DirectSearchInit, GetGoalCameraPose, FrustumViz
from cropbox_search.record_states import GridInitStateRecording
from common.move import MovePTU
from common.visualize_waypoints import VisualizeWaypoints

class GridInitRecordStateMachine():

    sm_grid_init_record = smach.StateMachine(outcomes=['aborted',
                                                       'nowhere_left_to_search'],
                                             input_keys=['searched_object_types_and_ids'])

    with sm_grid_init_record:

        smach.StateMachine.add('GET_GOAL_CAMERA_POSE',
                               GetGoalCameraPose(),
                               transitions={'MoveRobot':'MOVE_ROBOT',
                                            'MovePTUOnly':'MOVE_PTU_ONLY',
                                            'nowhere_left_to_search':'nowhere_left_to_search',
                                            'aborted':'aborted'},
                               remapping={'goal_robot_pose':'goal_robot_pose',
                                          'goal_ptu_position':'goal_ptu_position',
                                          'searched_object_types_and_ids':'searched_object_types_and_ids',
                                          'filtered_searched_object_types':'filtered_searched_object_types'})

        smach.StateMachine.add('MOVE_ROBOT',
                               GetMoveRobotSateMachine(),
                               transitions={'succeeded':'VISUALIZE_FRUSTUM',
                                            'aborted':'aborted'},
                               remapping={'goal_robot_pose':'goal_robot_pose',
                                          'goal_ptu_position':'goal_ptu_position'})

        smach.StateMachine.add('MOVE_PTU_ONLY',
                               MovePTU(),
                               transitions={'succeeded':'VISUALIZE_FRUSTUM',
                                            'aborted':'aborted'},
                               remapping={'goal_ptu_position':'goal_ptu_position'})

        smach.StateMachine.add('VISUALIZE_FRUSTUM',
                               FrustumViz(),
                               transitions={'succeeded':'VISUALIZE_WAYPOINT'})

        smach.StateMachine.add('VISUALIZE_WAYPOINT',
                               VisualizeWaypoints(),
                               transitions={'succeeded':'STATE_RECORDING'})

        smach.StateMachine.add('STATE_RECORDING',
                               GridInitStateRecording(),
                               transitions={'succeeded':'GET_GOAL_CAMERA_POSE',
                                            'aborted':'aborted'},
                               remapping={'goal_robot_pose':'goal_robot_pose',
                                          'goal_ptu_position':'goal_ptu_position'})


    sm_grid_init_record_search = smach.StateMachine(outcomes=['aborted',
                                                              'nowhere_left_to_search'])

    with sm_grid_init_record_search:
        smach.StateMachine.add('GRID_INIT_RECORD_INIT',
                               DirectSearchInit(),
                               transitions={'object_poses_from_demonstrations_given':'GRID_INIT_RECORD',
                                            'no_object_poses_from_demonstrations':'GRID_INIT_RECORD',
                                            'aborted':'aborted'},
                               remapping={'searched_object_types_and_ids':'searched_object_types_and_ids'})

        smach.StateMachine.add('GRID_INIT_RECORD',
                               sm_grid_init_record,
                               transitions={'aborted':'aborted',
                                            'nowhere_left_to_search':'nowhere_left_to_search'},
                               remapping={'searched_object_types_and_ids':'searched_object_types_and_ids'})


