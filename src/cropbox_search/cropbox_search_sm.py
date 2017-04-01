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

from common.common_sm import GetRelationBasedSearchStateMachine
from common.init import SearchInit
from indirect_search.ism import SceneRecognition
from indirect_search.nbv import NBVSetPointCloud
from cropbox_search.cropbox_search_states import CropBoxGeneration

class CropBoxSearchStateMachine():

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
                                            'too_many_deactivated_normals' : 'aborted'},
                               remapping={'object_pointcloud':'object_pointcloud'})



    sm_cropbox_search = smach.StateMachine(outcomes=['aborted',
                                                     'found_all_objects',
                                                     'found_all_required_scenes',
                                                     'nowhere_left_to_search'])

    with sm_cropbox_search:
        smach.StateMachine.add('CROPBOX_SEARCH_INIT',
                               sm_init,
                               transitions={'succeeded':'CROPBOX_SEARCH',
                                            'aborted':'aborted'})

        smach.StateMachine.add('CROPBOX_SEARCH',
                               GetRelationBasedSearchStateMachine(),
                               transitions={'found_objects':'SCENE_RECOGNITION',
                                            'aborted':'aborted',
                                            'no_nbv_found':'nowhere_left_to_search'})

        smach.StateMachine.add('SCENE_RECOGNITION',
                               SceneRecognition(),
                               transitions={'found_scenes':'CROPBOX_SEARCH',
                                            'found_all_required_scenes':'found_all_required_scenes',
                                            'found_all_objects':'found_all_objects',
                                            'aborted':'aborted'})

