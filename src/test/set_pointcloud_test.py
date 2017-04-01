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
import numpy
import tf
import math

from asr_next_best_view.srv import SetAttributedPointCloud
from asr_next_best_view.msg import RobotStateMessage
from asr_msgs.msg import AsrAttributedPointCloud, AsrAttributedPoint
from geometry_msgs.msg import Point, Quaternion, Twist


def main():
    rospy.init_node('Test_setpointcloud')
    while not rospy.is_shutdown():
        point_cloud = AsrAttributedPointCloud()

        for x in range(-50, 50):
           for y in range(-50, 50):
                point = AsrAttributedPoint()
                point.pose.position = Point(*[float(x/10.0), float(y/10.0), float(0.8)])
                point.pose.orientation = Quaternion(*[1, 0, 0, 1])
                point.type = 'Smacks'
                point.identifier = '0'
                point_cloud.elements.append(point)
                print point

        try:
            rospy.wait_for_service('/nbv/set_point_cloud', timeout=5)
        except rospy.exceptions.ROSException, e:
            rospy.loginfo("Couldn't reach service")
            rospy.signal_shutdown('Service call unreachable')

        set_pc = rospy.ServiceProxy('/nbv/set_point_cloud',
                                    SetAttributedPointCloud)
        set_pc_rsp = set_pc(point_cloud)

        if set_pc_rsp.is_valid == False:
            rospy.signal_shutdown('Service call unsucessful')
        else:
            return 0

    rospy.spin()


if __name__ == '__main__':
    main()
