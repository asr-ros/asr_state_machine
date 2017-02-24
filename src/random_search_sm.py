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
from states.object_detection import ObjectDetection
from states.ism import SceneRecognition
from states.random_search_states import GenerateRandomPose
from states.visualize_waypoints import VisualizeWaypoints


class RandomSearchStateMachine():

    sm_random_search = smach.StateMachine(outcomes=['aborted',
                                                    'found_all_objects',
                                                    'found_all_required_scenes'])

    with sm_random_search:
        smach.StateMachine.add('RANDOM_SEARCH_INIT',
                               SearchInit(),
                               transitions={'succeeded':'GENERATE_RANDOM_POSE',
                                            'aborted':'aborted'},
                               remapping={'searched_object_types':'searched_object_types'})

        smach.StateMachine.add('GENERATE_RANDOM_POSE',
                               GenerateRandomPose(),
                               transitions={'succeeded':'MOVE_ROBOT'},
                               remapping={'goal_robot_pose':'goal_robot_pose',
                                          'goal_ptu_position':'goal_ptu_position'})

        smach.StateMachine.add('MOVE_ROBOT',
                               GetMoveRobotSateMachine(),
                               transitions={'succeeded':'VISUALIZE_WAYPOINT',
                                            'aborted':'GENERATE_RANDOM_POSE'},
                               remapping={'goal_robot_pose':'goal_robot_pose',
                                          'goal_ptu_position':'goal_ptu_position'})

        smach.StateMachine.add('VISUALIZE_WAYPOINT',
                               VisualizeWaypoints(),
                               transitions={'succeeded':'OBJECT_DETECTION'})

        smach.StateMachine.add('OBJECT_DETECTION',
                               ObjectDetection(),
                               transitions={'found_objects':'SCENE_RECOGNITION',
                                            'no_objects_found':'GENERATE_RANDOM_POSE',
                                            'aborted':'aborted'},
                               remapping={'searched_object_types':'searched_object_types',
                                          'detected_objects':'detected_objects'})

        smach.StateMachine.add('SCENE_RECOGNITION',
                               SceneRecognition(),
                               transitions={'found_scenes':'GENERATE_RANDOM_POSE',
                                            'found_all_required_scenes':'found_all_required_scenes',
                                            'found_all_objects':'found_all_objects',
                                            'aborted':'aborted'})

  
