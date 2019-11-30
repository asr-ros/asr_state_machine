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
from asr_msgs.msg import AsrObject
from asr_world_model.srv import PushFoundObject, PushFoundObjectList
import asr_aruco_marker_recognition.srv 
import asr_fake_object_recognition.srv
from asr_world_model.srv import GetRecognizerName
import asr_descriptor_surface_based_recognition.srv 

class ObjectDetectorsManager:

    def __init__(self):
        pass

    def start_recognizers_descriptor(self, searched_object_types):
        try:
            rospy.wait_for_service(
                    '/asr_descriptor_surface_based_recognition/get_recognizer',
                    timeout=3)
            descriptor_recognizer = rospy.ServiceProxy(
                    '/asr_descriptor_surface_based_recognition/get_recognizer',
                    asr_descriptor_surface_based_recognition.srv.GetRecognizer)
        except (rospy.exceptions.ROSException, rospy.ServiceException) as e:
                rospy.logwarn("Service error with descriptor recognizer")
                return 'aborted'
        descriptor_recognizer(searched_object_types, 1, False)
        rospy.loginfo('Descriptor_surface_recognition started for ' + str(searched_object_types))
        return 'succeeded'

    def start_markers(self):
        try:
            rospy.wait_for_service(
                    '/asr_aruco_marker_recognition/get_recognizer',
                    timeout=3)
            marker_recognizer = rospy.ServiceProxy(
                    '/asr_aruco_marker_recognition/get_recognizer',
                    asr_aruco_marker_recognition.srv.GetRecognizer)
            marker_recognizer()
        except (rospy.exceptions.ROSException, rospy.ServiceException) as e:
            rospy.logwarn("Service error with marker recognizer")
            return 'aborted'
        rospy.loginfo('Marker recognition started')
        return 'succeeded'

    def stop_recognizers_descriptor(self, searched_object_types):
        release_descriptor_recognizer = None
        try:
            release_descriptor_recognizer = rospy.ServiceProxy(
                    '/asr_descriptor_surface_based_recognition/release_recognizer',
                    asr_descriptor_surface_based_recognition.srv.ReleaseRecognizer)
        except rospy.ServiceException, e:
            rospy.logwarn("Error calling the release recognizer services for descriptor recognition.")
            return 'aborted'
        release_descriptor_recognizer(searched_object_types)
        rospy.loginfo("Recognition of "+str(searched_object_types)+" released for descriptor_surface_based.")
        return 'succeeded'

    def stop_markers(self):
        try:
            release_marker_recognizer = rospy.ServiceProxy(
                    '/asr_aruco_marker_recognition/release_recognizer',
                    asr_aruco_marker_recognition.srv.ReleaseRecognizer)
        except rospy.ServiceException, e:
            rospy.logwarn("Error calling the release recognizer services for marker recognition.")
            return 'aborted'
        release_marker_recognizer()
        rospy.loginfo("Marker recognition released.")
        return 'succeeded'

    def start_recognizers_sim(self,searched_object_types):
        fake_recognizer = None
        try:
            fake_recognizer = rospy.ServiceProxy(
                '/asr_fake_object_recognition/get_recognizer',
                asr_fake_object_recognition.srv.GetRecognizer)

        except rospy.ServiceException, e:
            rospy.logwarn("Error calling \'get\' fake based recognizer service.")
            return 'aborted'
        for object in searched_object_types:
            fake_recognizer(str(object))
        rospy.loginfo('Fake_object_recognition started for ' + str(searched_object_types))
        return 'succeeded'


    def stop_recognizers_sim(self,searched_object_types):
        fake_recognizer = None
        try:
            fake_recognizer = rospy.ServiceProxy(
                '/asr_fake_object_recognition/release_recognizer',
                asr_fake_object_recognition.srv.ReleaseRecognizer)

        except rospy.ServiceException, e:
            rospy.logwarn("Error calling \'release\' fake based recognizer service.")
            return 'aborted'
        for object in searched_object_types:
            fake_recognizer(str(object))
        rospy.loginfo("Recognition of "+str(searched_object_types)+" released for asr_fake_object_recognition.")
        return 'succeeded'


    def start_recognizers(self,searched_object_types):

        rospy.loginfo('Preparing to start recognizers for objects ' + str(searched_object_types))
        if rospy.get_param("/scene_exploration_sm/use_sensors") is True:
            try:
                recognizer_name_call = rospy.ServiceProxy(
                    '/env/asr_world_model/get_recognizer_name',
                    GetRecognizerName)
            except rospy.ServiceException, e:
                rospy.logwarn("Error calling get recognizer name service.")
                return 'aborted'

            marker_started = False
            for object in searched_object_types:
                if 'marker' in object:
                    if marker_started is False:
                        rospy.loginfo('Calling start marker recognition.')
                        self.start_markers()
                        marker_started = True
                else:
                    recognizer_name = recognizer_name_call(str(object)).recognizer_name
                    if str(recognizer_name) == 'descriptor':    
                        rospy.loginfo('Calling descriptor recognition for object ' + object)
                        self.start_recognizers_descriptor(object)
                    else:
                        rospy.logwarn("Error using unknown recognizer name " + str(recognizer_name))
                        return 'aborted'
        else:
            return self.start_recognizers_sim(searched_object_types)

    def stop_recognizers(self,searched_object_types):

        rospy.loginfo('Preparing to stop recognizers for objects ' + str(searched_object_types))
        if rospy.get_param("/scene_exploration_sm/use_sensors") is True:
            try:
                recognizer_name_call = rospy.ServiceProxy(
                    '/env/asr_world_model/get_recognizer_name',
                    GetRecognizerName)
            except rospy.ServiceException, e:
                rospy.logwarn("Error calling get recognizer name service.")
                return 'aborted'

            marker_stopped = False
            for object in searched_object_types:
                if 'marker' in object:
                    if marker_stopped is False:
                        rospy.loginfo('Calling stop marker recognition.')
                        self.stop_markers()
                        marker_stopped = True
                else:
                    recognizer_name = recognizer_name_call(str(object)).recognizer_name
                    if str(recognizer_name) == 'descriptor':
                        rospy.loginfo("Calling release descriptor for object " + object)
                        self.stop_recognizers_descriptor(object)
                    else:
                        rospy.loginfo("aborted")
                        return 'aborted'
        else:
            return self.stop_recognizers_sim(searched_object_types)
