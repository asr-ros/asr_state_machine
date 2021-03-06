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

from pose_sampling import *

# To import files from parten directories
if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))

from common.move import * 
from indirect_search.ism import *
from indirect_search.nbv import *
from common.object_detection import *
from common.init import *


class ObjectDetectionInit(smach.State):
    def __init__(self):
        smach.State.__init__(
                self,
                outcomes=['succeeded', 'aborted'],
                output_keys=['searched_object_types'])

    def execute(self, userdata):
        userdata.searched_object_types = ['Smacks', 'Amicelli', 'Marker1']
        return 'succeeded'

def main():
    rospy.init_node('object_detection_sm')

    sm_object_detection = smach.StateMachine(outcomes=['succeeded', 'aborted',
                                                       'no_objects_found'])

    with sm_object_detection:
        smach.StateMachine.add("INIT",
                               ObjectDetectionInit(),
                               transitions={'succeeded': 'OBJECT_DETECTION'})

        smach.StateMachine.add('OBJECT_DETECTION',
                               ObjectDetection(),
                               transitions={'no_objects_found':'no_objects_found',
                                            'found_objects':'succeeded',
                                            'aborted':'aborted'},
                               remapping={'searched_object_types':'searched_object_types',
                                          'detected_objects':'detected_objects'})

    server = smach_ros.IntrospectionServer(
        'object_detection_test_sm',
        sm_object_detection,
        '/OBJECT_DETECTION_TEST_SOBJECT_DETECTION_TEST_SM')
    server.start()
    # smach.set_preempt_hanlder(sm_main)
    smach_thread = threading.Thread(target = sm_object_detection.execute)
    smach_thread.start()
    rospy.spin()
    server.stop()
    rospy.signal_shutdown('All done.')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
