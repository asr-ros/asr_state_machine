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
import json
import random
import shutil
import csv
import time

from actionlib import *
from actionlib.msg import *
import sensor_msgs.msg
import math
from indirect_search_sm import IndirectSearchStateMachine
from direct_search_sm import DirectSearchStateMachine
from random_search_sm import RandomSearchStateMachine
from cropbox_search_sm import CropBoxSearchStateMachine
from cropbox_record_sm import CropBoxRecordStateMachine
from grid_init_record_sm import GridInitRecordStateMachine
import states.init
import __builtin__ # hack for sharing log dir
import signal
import subprocess
from os.path import expanduser
from recognition_for_grasping.srv import ClearAllRecognizers as GraspingClearAllRecognizers
from asr_fake_object_recognition.srv import ClearAllRecognizers as FakeObjectClearAllRecognizers
from asr_descriptor_surface_based_recognition.srv import ClearAllRecognizers as DescriptorSurfaceClearAllRecognizers
from asr_aruco_marker_recognition.srv import ReleaseRecognizer
from asr_visualization_server.srv import DrawAllModelsMild
from states.state_acquisition import GetRobotState

# Naming convention
#   State names are all capitalized with underscores, e.g. MVU_PTU
#   State names that are (sub-) state machines start with SM_
#   State variables are lowercase separated by underscores
#   Classes follow the CapWords convention, e.g. MoveBase
#   Method names shoud be lowercase separated by underscores
#
#   remapping should always be defined even if its identical

global rosbag_proc

def main():
    """
    Start automata and introspection server
    """

    #config variables that should be outsourced in production TODO
    rospy.init_node('scene_exploration_sm')

    # Define before on_shutdown -> so that they are known
    global rosbag_proc
    rosbag_proc = None
    global smach_server
    smach_server = None


    # register closing callback
    rospy.on_shutdown(shutdown_hook)

    # NOTE: Writing variables in __builtin__ is bad and I don't encourage it,
    # but we need to access the log_dir in several other modules

    # Set True if you want to wait for key press after states
    __builtin__.evaluation = False

    log_dir = states.init.create_log_folder()
    __builtin__.log_dir = log_dir

    # Record to rosbag
    rosbag_proc = states.init.start_rosbag(log_dir)
    # Add 3d models to labor map
    rospy.loginfo("Waiting for visualization_server-service to become available")
    rospy.wait_for_service('visualization/draw_all_models_mild')
    try:
        draw_all_models_mild = rospy.ServiceProxy('visualization/draw_all_models_mild', DrawAllModelsMild)
        draw_all_models_mild()
        rospy.loginfo("Publishing room model")
    except rospy.ServiceException, e:
        rospy.logwarn("Error calling the draw all models mild service")


    # get initial objects from user
    # sleep just to wait till all init output is done
    rospy.sleep(1)
    initial_objects = states.init.get_initial_objects()
    rospy.loginfo("initial_objects: " + str(initial_objects))


    # write start time to logfile
    with open(__builtin__.log_dir +  '/log.csv', 'a') as log:
        logwriter = csv.writer(log, delimiter=' ',
                               quotechar='|', quoting=csv.QUOTE_MINIMAL)

        logwriter.writerow([time.time()])

    rospy.loginfo("Initial robot state:\n" + str(GetRobotState().get_robot_state()))

    #Decide with which state machine (e.g. from above or from included modules) we want to start searching.
    rospy.loginfo("We are running our state machine in the following mode:")

    mode = rospy.get_param("/scene_exploration_sm/mode")
    if mode is not 1 and mode is not 2 and mode is not 3 and mode is not 4 and mode is not 5 and mode is not 6 and mode is not 7:
        #Make informative search the default search
        rospy.logwarn("Unknown mode was set in params -> informative search will be taken as default")
        rospy.set_param("/scene_exploration_sm/mode", 2)
        mode = 2


    if mode is 1 :
        rospy.loginfo("Direct Search mode as stand alone")
        sm_direct_search_as_standalone = DirectSearchStateMachine().sm_direct_search_as_standalone
        smach_server = startIntrospectionServer(sm_direct_search_as_standalone, '/DIRECT_SEARCH')
        smach_thread = threading.Thread(target = sm_direct_search_as_standalone.execute)
    elif mode is 2 :
        rospy.loginfo("Indirect Search mode as stand alone")
        sm_indirect_search = IndirectSearchStateMachine().get_indirect_sm_as_stand_alone(initial_objects)
        smach_server = startIntrospectionServer(sm_indirect_search, '/INDIRECT_SEARCH')
        smach_thread = threading.Thread(target=sm_indirect_search.execute)
    elif mode is 3:
        rospy.loginfo("Object Search mode")

        #Meta state machine containing direct- and indirect search
        sm_object_search = smach.StateMachine(outcomes=['aborted',
                                                        'found_all_objects',
                                                        'found_all_required_scenes',
                                                        'nowhere_left_to_search'])
        # construct meta state machine
        with sm_object_search:
            smach.StateMachine.add('OBJECT_SEARCH_INIT',
                                   states.init.ObjectSearchInit(),
                                   transitions={'succeeded':'DIRECT_SEARCH',
                                                'aborted':'aborted'})

            smach.StateMachine.add('DIRECT_SEARCH',
                                   DirectSearchStateMachine().sm_direct_search_for_object_search,
                                   transitions={'found_objects':'INDIRECT_SEARCH',
                                                'aborted':'aborted',
                                                'nowhere_left_to_search':'nowhere_left_to_search'})

            smach.StateMachine.add('INDIRECT_SEARCH',
                                   IndirectSearchStateMachine().get_indirect_sm_for_object_search(),
                                   transitions={'no_predictions_left':'DIRECT_SEARCH',
                                                'aborted':'aborted',
                                                'found_all_objects':'found_all_objects',
                                                'found_all_required_scenes':'found_all_required_scenes'})

        smach_server = startIntrospectionServer(sm_object_search, '/OBJECT_SEARCH')
        smach_thread = threading.Thread(target=sm_object_search.execute)
    elif mode is 4:
        rospy.loginfo("Random search mode")
        sm_random_search = RandomSearchStateMachine.sm_random_search
        smach_server = startIntrospectionServer(sm_random_search, '/RANDOM_SEARCH')
        smach_thread = threading.Thread(target=sm_random_search.execute)
    elif mode is 5:
        rospy.loginfo("Cropbox search mode")
        sm_cropbox_search = CropBoxSearchStateMachine().sm_cropbox_search
        smach_server = startIntrospectionServer(sm_cropbox_search, '/CROPBOX_SEARCH')
        smach_thread = threading.Thread(target=sm_cropbox_search.execute)
    elif mode is 6:
        rospy.loginfo("Cropbox record mode")
        sm_cropbox_record = CropBoxRecordStateMachine().sm_cropbox_record
        smach_server = startIntrospectionServer(sm_cropbox_record, '/CROPBOX_RECORD')
        smach_thread = threading.Thread(target=sm_cropbox_record.execute)
    elif mode is 7:
        rospy.loginfo("Grid init record mode")
        sm_grid_init_record_search = GridInitRecordStateMachine().sm_grid_init_record_search
        smach_server = startIntrospectionServer(sm_grid_init_record_search, '/GRID_INIT_RECORD')
        smach_thread = threading.Thread(target=sm_grid_init_record_search.execute)

    #Start searching
    smach_thread.start()
    rospy.spin()


def startIntrospectionServer(sm, nameOfSearch):
    if rospy.get_param("/scene_exploration_sm/enableSmachViewer"):
        rospy.loginfo("Smach viewer enabled")
        # Construct graphical output of SMACH automata.
        server = smach_ros.IntrospectionServer(
            'scene_exploration_state_machine',
            sm,
            nameOfSearch)
        #Start logging SMACH automat
        server.start()
        return server
    else:
        rospy.loginfo("Smach viewer disabled")
        return None

def shutdown_hook():
    rospy.loginfo("Enter shutdown_hook")
    """ Close rosbag recording, copy nbv logs and leave"""
    global rosbag_proc
    if rosbag_proc is not None:
        rospy.loginfo("Stop rosbag_proc")
        rosbag_proc.send_signal(subprocess.signal.SIGINT)

    #Stop logging SMACH automata
    global smach_server
    if smach_server is not None:
        rospy.loginfo("Stop smach_server")
        smach_server.stop()

    if __builtin__.log_dir:
        home = expanduser("~")
        try:
            shutil.copy(home + "/log/nbv.log", __builtin__.log_dir)
        except IOError:
            pass
        try:
            shutil.copy(home + "/log/asr_world_model.log", __builtin__.log_dir)
        except IOError:
            pass
        try:
            shutil.copy(home + "/log/ism.log", __builtin__.log_dir)
        except IOError:
            pass
        try:
            shutil.copy(home + "/log/state_machine.log", __builtin__.log_dir)
        except IOError:
            pass
        try:
            shutil.copy(home + "/log/direct_search_manager.log", __builtin__.log_dir)
        except IOError:
            pass

    #Stop recognizers in case signal is caught during object detection.
    if rospy.get_param("/scene_exploration_sm/use_sensors") is False:
        try:
            release_fake_recognizers = rospy.ServiceProxy('/asr_fake_object_recognition/clear_all_recognizers', FakeObjectClearAllRecognizers)
            release_fake_recognizers()
        except rospy.ServiceException, e:
            rospy.logwarn("Error calling the clear for /asr_fake_object_recognition/clear_all_recognizers service: " + str(e))
    else:
        try:
            release_tex_seg_recognizers = rospy.ServiceProxy('/recognition_manager/clear_all_recognizers', GraspingClearAllRecognizers)
            release_tex_seg_recognizers("/stereo/objects")
        except rospy.ServiceException, e:
            rospy.logwarn("Error calling the clear for recognition_manager/clear_all_recognizers service: " + str(e))
        try:
            release_desc_recognizers = rospy.ServiceProxy('/asr_descriptor_surface_based_recognition/clear_all_recognizers', DescriptorSurfaceClearAllRecognizers)
            release_desc_recognizers()
        except rospy.ServiceException, e:
            rospy.logwarn("Error calling the clear for /asr_descriptor_surface_based_recognition/clear_all_recognizers service: " + str(e))
        try:
            release_marker_recognizers = rospy.ServiceProxy('/asr_aruco_marker_recognition/release_recognizer', ReleaseRecognizer)
            release_marker_recognizers()
        except rospy.ServiceException, e:
            rospy.logwarn("Error calling the clear for /asr_aruco_marker_recognition/release_recognizer service: " + str(e))

    rospy.loginfo("Finished shutdown_hook")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
