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
import numpy
from time import time
import math

from geometry_msgs.msg import (Pose, Point, Quaternion)
from next_best_view.msg import RobotStateMessage
from asr_robot_model_services.srv import GetPose
from sensor_msgs.msg import JointState
import tf
import tf2_ros

"""
Methods that are required for calculating and comparing the poses and states related to our robot during exploration. 
"""

"""
Accessing tf via c++ since python access is not reliable.
"""
def get_camera_pose_cpp():
    """
    Returns camera pose
    """
    rospy.wait_for_service('/asr_robot_model_services/GetCameraPose', timeout=5)
    pose = rospy.ServiceProxy('/asr_robot_model_services/GetCameraPose',GetPose)
    return pose().pose

"""
Old helper for python interface. Might be still used, know wraps c++ interface.
"""
def get_camera_pose():
    """
    Returns current camera pose
    """
    return get_camera_pose_cpp()
    # tf2 is used since tf had some strange behaviour.
    # If there are still errors in this place, look deeper in the ptu driver
    # since the tf problem is happening there.
    #tfBuffer = tf2_ros.Buffer()
    #listener = tf2_ros.TransformListener(tfBuffer)
    
    #rospy.sleep(1.0)
    #try:
    #    trans = tfBuffer.lookup_transform("map", "ptu_mount_link", rospy.Time(0),
    #                                      rospy.Duration(2))
    #    return Pose(trans.transform.translation,
    #                trans.transform.rotation)
                    
    #except Exception, e:
    #    rospy.logwarn("Couldn't get Camera pose from tf2")
    #   rospy.logwarn(e)

"""
Accessing tf via c++ since python access is not reliable.
"""
def get_robot_pose_cpp():
    """
    Returns robot pose
    """
    rospy.wait_for_service('/asr_robot_model_services/GetRobotPose', timeout=5)
    pose = rospy.ServiceProxy('/asr_robot_model_services/GetRobotPose',GetPose)
    return pose().pose

"""
Old helper for python interface. Might be still used, know wraps c++ interface.
"""
def get_robot_pose():
    """
    Returns robot pose
    """
    return get_robot_pose_cpp()
    #listener = tf.TransformListener()
    #listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(4.0))
    #try:
        ##now = rospy.Time.now()
        ##listener.waitForTransform("/map", "/base_link", now, rospy.Duration(4.0))
        #(trans, rot) = listener.lookupTransform("/map", "/base_link",
                                                #rospy.Time(0))
        #return Pose(Point(*trans), Quaternion(*rot))
    #except (tf.LookupException, tf.ConnectivityException,
            #tf.ExtrapolationException), e:
        #rospy.logwarn(str(e))
        #rospy.logwarn("Could not get tf transformation for base link pose")

def get_rotation_from_pose(pose):
    """
    Returns the rotation in degrees from pose
    """
    pose.position.x = pose.position.x
    pose.position.y = pose.position.y
    pose.position.z = 0
    q = numpy.array([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w])
    (_, _, yaw) = tf.transformations.euler_from_quaternion(q)
    return yaw

class GetRobotState(smach.State):

    ptu = None

    def __init__(self):
        pass

    def ptu_callback(self, data):
        self.ptu = data.position

    def get_robot_state(self):
        """
        Returns the current robot configuration consisting of position and ptu
        angles
        """

        sub_ptu = rospy.Subscriber('/ptu_driver/state', JointState, self.ptu_callback)

        future = time() + 2
        while not self.ptu and time() < future:
            pass

        rospy.sleep(2)
        if not self.ptu:
            self.ptu = [0,0]

        pose = get_robot_pose_cpp()

        robot_state = RobotStateMessage()
        robot_state.pan = self.ptu[0]
        robot_state.tilt = self.ptu[1]
        robot_state.x = pose.position.x
        robot_state.y = pose.position.y
        robot_state.rotation = get_rotation_from_pose(pose)

        return robot_state

def approx_equal_robot_state(old_robot_pose, old_tilt, new_robot_pose, new_tilt, positionThreshold, orientationThreshold, tiltThreshold):
    """
    Checks if two robot states are approximately equal, i.e. disctance of all values are under
    a threshold.
     """

    if not old_robot_pose or not old_tilt or not new_robot_pose or not new_tilt: return False

    positionDiff = euclidean_distance_3d(old_robot_pose.position, new_robot_pose.position)
    orientationDiffInDegree = angle_between_quaternions(old_robot_pose.orientation, new_robot_pose.orientation)
    tiltDiff = abs(old_tilt - new_tilt)

    if abs(positionDiff) < positionThreshold and abs(orientationDiffInDegree) < orientationThreshold and tiltDiff < tiltThreshold:
        rospy.loginfo("Two robot states are approx equale.")
#        rospy.loginfo("old_robot_pose:\n" + str(old_robot_pose))
#        rospy.loginfo("new_robot_pose:\n" + str(new_robot_pose))
#        rospy.loginfo("Difference positionDiff: " + str(positionDiff))
#        rospy.loginfo("Difference orientationDiffInDegree: " + str(orientationDiffInDegree))
#        rospy.loginfo("Difference tilfDiff: " + str(tiltDiff))
        return True

    return False

def approx_equal_pose(old_pose, new_pose, threshold):
    """
     Checks if two poses are approximately equal, i.e. sum of all values is under
     a specified threshold.
     """

    if not old_pose or not new_pose: return False

    diff = euclidean_distance_3d(old_pose.position, new_pose.position) + \
           angle_between_quaternions(old_pose.orientation, new_pose.orientation)
    rospy.loginfo("Difference poses: " + str(diff))
    return diff < threshold


def euclidean_distance_3d(position1, position2):
    return math.sqrt(
        pow(position1.x - position2.x, 2) + \
        pow(position1.y - position2.y, 2) + \
        pow(position1.z - position2.z, 2))

def angle_between_quaternions(quaternion1, quaternion2):
    quaternionList1 = quaternion_to_list(quaternion1)
    quaternionList2 = quaternion_to_list(quaternion2)
    # convert quaternions to vectors
    vector1 = get_vector(quaternionList1)
    vector2 = get_vector(quaternionList2)

    return angle_between_vectors(vector1, vector2)


def angle_between_vectors(vector1, vector2):
    if numpy.linalg.norm(vector1) * numpy.linalg.norm(vector2) <= 0.00001:
        return 0.0

    cosOfAngle = numpy.dot(vector1, vector2) / (numpy.linalg.norm(vector1) * \
                                                numpy.linalg.norm(vector2))

    if cosOfAngle < -1.0: cosOfAngle = -1.0
    if cosOfAngle > 1.0: cosOfAngle = 1.0

    rad = math.acos(cosOfAngle)

    return math.degrees(rad)


def get_vector(quaternion):
    q = [1, 0, 0, 0]
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(quaternion, q),
        tf.transformations.quaternion_conjugate(quaternion)
    )[:3]


def quaternion_to_list(quaternion):
    return [quaternion.x, quaternion.y, quaternion.z, quaternion.w]

