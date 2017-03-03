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
import time
import time
import timeit

from geometry_msgs.msg import (Pose, Point, Vector3)
from asr_next_best_view.srv import TriggerFrustumVisualization
from evaluation_decorators import *
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import state_acquisition
import __builtin__

def visualize_waypoints(current_robot_pose, current_camera_pose,
                        marker_id, last_robot_pose=None):
    """
    Publishes marker to visualize robot path
    """

    waypoints = rospy.get_param("/scene_exploration_sm/waypoints")
    publisher = rospy.Publisher(waypoints, Marker, queue_size=10)
    points = []
    rospy.logdebug("last_robot_pose is: " + str(last_robot_pose))
    if last_robot_pose is not None:
        points.append(Point(last_robot_pose.position.x,
                            last_robot_pose.position.y,
                            0))
        points.append(Point(current_robot_pose.position.x,
                            current_robot_pose.position.y,
                            0))

    if current_camera_pose is not None:
        z_pose = current_camera_pose.position.z
    else:
        z_pose = 1.35

    center = Point(current_robot_pose.position.x,
                   current_robot_pose.position.y,
                   z_pose / 2.0)
    pose_marker = Marker()
    pose_marker.header.stamp = rospy.Time.now()
    pose_marker.header.frame_id = '/map'
    pose_marker.ns = 'waypoints_cyl'
    pose_marker.type = Marker.CYLINDER
    pose_marker.id = marker_id + 2
    pose_marker.action = Marker.ADD
    pose_marker.scale = Vector3(0.05, 0.05, z_pose)
    pose_marker.color = ColorRGBA(0, 0, 1, 1)
    pose_marker.lifetime = rospy.Duration()
    pose_marker.pose.position = center

    # Only on the first try we have to wait for the publisher,
    # next times we know the last pose and this won't be executed
    if last_robot_pose is None:
        rospy.loginfo("Sleeping till waypoint publisher is ready")
        rospy.sleep(1)

    publisher.publish(pose_marker)

    marker = Marker()
    marker.header.stamp = rospy.Time.now()
    marker.header.frame_id = '/map'
    marker.ns = 'waypoints_lines'
    marker.type = Marker.LINE_LIST
    marker.id = marker_id
    marker.action = Marker.ADD
    marker.scale = Vector3(0.02, 0.1, 0.1)
    marker.color = ColorRGBA(0, 1, 1, 1)
    marker.lifetime = rospy.Duration()
    marker.points = points

    publisher.publish(marker)

    if current_camera_pose is not None:
        arrow = Marker()
        arrow.header.stamp = rospy.Time.now()
        arrow.header.frame_id = '/map'
        arrow.ns = 'waypoints_arrows'
        arrow.pose = current_camera_pose
        arrow.type = Marker.ARROW
        arrow.id = marker_id + 1
        arrow.action = Marker.ADD
        arrow.scale = Vector3(0.5, 0.02, 0.02)
        arrow.color = ColorRGBA(1, 1, 0, 1)
        arrow.lifetime = rospy.Duration()

        publisher.publish(arrow)

# Make them independent from class, so that in- and direct search will not override each others waypoints
global waypoint_marker_id
waypoint_marker_id = 0
global waypoint_old_pose
waypoint_old_pose = None

class VisualizeWaypoints(smach.State):
    def __init__(self):
        smach.State.__init__(
            self,
            outcomes=['succeeded'])

    @timed
    @log
    def execute(self, userdata):
        rospy.loginfo('Executing VISUALIZE_WAYPOINTS')
        current_pose = state_acquisition.get_robot_pose_cpp()
        camera_pose = state_acquisition.get_camera_pose_cpp()

        global waypoint_marker_id
        global waypoint_old_pose

        if (current_pose and camera_pose):
            visualize_waypoints(current_pose, camera_pose,
                                waypoint_marker_id, waypoint_old_pose)

            try:
                rospy.wait_for_service('/nbv/trigger_frustum_visualization', timeout=5)
                update_frustum = rospy.ServiceProxy(
                    '/nbv/trigger_frustum_visualization',
                    TriggerFrustumVisualization)
                update_frustum(camera_pose)
            except (rospy.ServiceException, rospy.exceptions.ROSException), e:
                rospy.logwarn(str(e))

            waypoint_old_pose = current_pose
            waypoint_marker_id += 4

        return 'succeeded'
