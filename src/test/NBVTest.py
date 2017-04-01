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

import roslib; roslib.load_manifest('scene_exploration')
import rospy
import smach
import smach_ros
import json
import random
import numpy as np
import tf
#from States import *
import __builtin__ # hack for sharing log dir

from pose_sampling import *

# To import files from parten directories
if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

import common.init
from common.move import * 
from indirect_search.ism import *
from indirect_search.nbv import *
from common.object_detection import *
from common.visualize_waypoints import *

"""
Simple test implementing the same test case than Ralfs NBV test in the NBV
package. It samples some points and moves to them to invalidate.
"""

def main():
    rospy.init_node('nbv_test_automat')

    log_dir = common.init.create_log_folder()
    __builtin__.log_dir = log_dir
    __builtin__.evaluation = False
    
    # submachine for movement (base + ptu)
    sm_move = smach.Concurrence(
                outcomes=['succeeded', 'aborted'],
                default_outcome='aborted',
                outcome_map={
                    'succeeded': {'MOVE_BASE': 'succeeded', 
                                  'MOVE_PTU': 'succeeded'}}, 
                input_keys=['goal_robot_pose', 'goal_ptu_position'])

    with sm_move:
        smach.Concurrence.add(
               'MOVE_PTU',
               MovePTU())

        smach.Concurrence.add(
                'MOVE_BASE',
                MoveBase())

    visualize_waypoints = VisualizeWaypoints()


    sm_object_search = smach.StateMachine(outcomes=['succeeded', 'aborted',
                                                    'no_nbv_found'],
                                          input_keys=['object_pointcloud'])

    with sm_object_search:
        # IN: object_point_cloud
        smach.StateMachine.add('NBV_SET_POINT_CLOUD',
                               NBVSetPointCloud(),
                               transitions={'succeeded':'NEXT_BEST_VIEW'},
                               remapping={'object_pointcloud': 'object_pointcloud'})
                     
        # OUT: goal_robot_pose, goal_ptu_position, goal_camera_pose
        smach.StateMachine.add('NEXT_BEST_VIEW',
                               NextBestView(),
                               transitions={'found_next_best_view': 'MOVE_ROBOT_TO_VIEW',
                                            'nbv_update_point_cloud':'NBV_UPDATE_POINT_CLOUD'},
                               remapping={'goal_camera_pose' : 'goal_camera_pose',
                                          'goal_robot_pose' : 'goal_robot_pose',
                                          'goal_ptu_position' : 'goal_ptu_position',
                                          'searched_object_types' : 'searched_object_types'})

        # IN: 
        smach.StateMachine.add('MOVE_ROBOT_TO_VIEW',
                               sm_move,
                               transitions={'succeeded': 'VISUALIZE_WAYPOINT'},
                               remapping={'goal_robot_pose': 'goal_robot_pose',
                                          'goal_ptu_position': 'goal_ptu_position'})

        smach.StateMachine.add('VISUALIZE_WAYPOINT',
                               visualize_waypoints,
                               transitions={'succeeded': 'OBJECT_DETECTION'})

        # smach.StateMachine.add('CURRENT_POSE',
        #                         CurrentPose(),
        #                         transitions={'succeeded': 'OBJECT_DETECTION'})

        smach.StateMachine.add('OBJECT_DETECTION',
                               ObjectDetection(),
                               transitions={'no_objects_found': 'NBV_UPDATE',
                                            'succeeded': 'succeeded'},
                               remapping={'searched_object_types': 'object'})

        smach.StateMachine.add('NBV_UPDATE',
                               NextBestViewUpdate(),
                               transitions={'succeeded': 'NEXT_BEST_VIEW'},
                               remapping={
                                   'remaining_attempts': 'remaining_attempts'})



        sm_main = smach.StateMachine(outcomes=['succeeded',
                                               'aborted','no_nbv_found'])
        with sm_main:


            smach.StateMachine.add('INIT',
                                   InitSceneExploration(), 
                                   transitions={'succeeded': 'VISUALIZE_WAYPOINT',
                                                'aborted': 'aborted'})

            smach.StateMachine.add('VISUALIZE_WAYPOINT',
                                   visualize_waypoints,
                                   transitions={'succeeded': 'POSE_SAMPLING'})

            smach.StateMachine.add('POSE_SAMPLING',
                                   PoseSampling(),
                                   remapping={'object_pointcloud':
                                              'object_pointcloud'},
                                   transitions={'succeeded': 'OBJECT_SEARCH',
                                                'aborted': 'aborted'})

            smach.StateMachine.add('OBJECT_SEARCH',
                                   sm_object_search,
                                   transitions={'succeeded': 'succeeded'})

        server = smach_ros.IntrospectionServer(
                    'nbv_test_automat',
                    sm_main,
                    '/NBV_TEST_SM')
        server.start()
        # smach.set_preempt_hanlder(sm_main)
        smach_thread = threading.Thread(target = sm_main.execute)
        smach_thread.start()
        rospy.spin()
        server.stop()
        rospy.signal_shutdown('All done.')


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
