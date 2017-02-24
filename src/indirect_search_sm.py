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

from common_sm import GetRelationBasedSearchStateMachine
from states.ism import SceneRecognition, PosePrediction
from states.nbv import NBVSetPointCloud
from states.object_detection import ObjectDetection
from states.init import SearchInit
from states.visualize_waypoints import VisualizeWaypoints

class IndirectSearchStateMachine():

    def get_indirect_sm_as_stand_alone(self, initial_objects):

        sm_indirect_serach_init = smach.StateMachine(outcomes=['succeeded',
                                                               'aborted'])
        with sm_indirect_serach_init:
            smach.StateMachine.add('SEARCH_INIT',
                                   SearchInit(initial_objects),
                                   transitions={'succeeded':'OBJECT_DETECTION',
                                                'aborted':'aborted'},
                                   remapping={'searched_object_types':'searched_object_types'})

            smach.StateMachine.add('OBJECT_DETECTION',
                                   ObjectDetection(),
                                   transitions={'found_objects':'VISUALIZE_WAYPOINT',
                                                'no_objects_found':'aborted',
                                                'aborted':'aborted'},
                                   remapping={'searched_object_types':'searched_object_types',
                                              'detected_objects':'detected_objects'})

            smach.StateMachine.add('VISUALIZE_WAYPOINT',
                                   VisualizeWaypoints(),
                                   transitions={'succeeded':'succeeded'})



        sm_indirect_search = smach.StateMachine(outcomes=['aborted',
                                                          'found_all_objects',
                                                          'found_all_required_scenes',
                                                          'no_predictions_left'])

        with sm_indirect_search:
            smach.StateMachine.add('INDIRECT_SEARCH_INIT',
                                   sm_indirect_serach_init,
                                   transitions={'succeeded':'INDIRECT_SEARCH',
                                                'aborted':'aborted'})

            smach.StateMachine.add('INDIRECT_SEARCH',
                                   self.get_indirect_sm_for_object_search(),
                                   transitions={'found_all_objects':'found_all_objects',
                                                'found_all_required_scenes':'found_all_required_scenes',
                                                'no_predictions_left':'no_predictions_left',
                                                'aborted':'aborted'})

        return sm_indirect_search




    def get_indirect_sm_for_object_search(self):


        sm_indirect_search = smach.StateMachine(outcomes=['aborted',
                                                          'found_all_objects',
                                                          'found_all_required_scenes',
                                                          'no_predictions_left'])

        with sm_indirect_search:
            smach.StateMachine.add('SCENE_RECOGNITION',
                                   SceneRecognition(),
                                   transitions={'found_scenes':'OBJECT_POSE_PREDICTION',
                                                'found_all_required_scenes':'found_all_required_scenes',
                                                'found_all_objects':'found_all_objects',
                                                'aborted':'aborted'})

            smach.StateMachine.add('OBJECT_POSE_PREDICTION',
                                   PosePrediction(),
                                   transitions={'succeeded':'NBV_SET_POINT_CLOUD',
                                                'aborted':'aborted',
                                                'no_predictions_left':'no_predictions_left'},
                                   remapping={'object_pointcloud':'object_pointcloud'})

            smach.StateMachine.add('NBV_SET_POINT_CLOUD',
                                   NBVSetPointCloud(),
                                   transitions={'succeeded':'RELATION_BASED_SEARCH',
                                                'aborted':'aborted',
                                                'too_many_deactivated_normals':'OBJECT_POSE_PREDICTION'},
                                   remapping={'object_pointcloud':'object_pointcloud'})


            smach.StateMachine.add('RELATION_BASED_SEARCH',
                                   GetRelationBasedSearchStateMachine(),
                                   transitions={'found_objects':'SCENE_RECOGNITION',
                                                'aborted':'aborted',
                                                'no_nbv_found':'OBJECT_POSE_PREDICTION'})

        return sm_indirect_search
