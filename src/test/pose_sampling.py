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
import random
import numpy as np
import tf
import math

from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from asr_msgs.msg import AsrAttributedPointCloud, AsrAttributedPoint

class PoseSampling(smach.State):
    """
    Generate sample pointcloud using normal distribution for defined accumulation
    points. 
    
    This calls is adapted from Ralfs NBV Test class.
    """

    def get_random_vector(self, mean, standard_deviaton):
        """ adapted from MathHelper in NBV """
        result = []
        for i in xrange(len(mean)):
            result.append(np.random.normal(mean[i], standard_deviaton[i]))
        return result

    def get_random_quaternion(self):
        """ adapted from MathHelper in NBV """
        random_angles = self.get_random_vector([0,0,0], [2*np.pi, 2*np.pi, 1])
        return tf.transformations.quaternion_from_euler(random_angles[0],
                                                 random_angles[1],
                                                 0)


    def __init__(self):
        smach.State.__init__(
                self,
                outcomes=['succeeded', 'aborted'],
                output_keys=['object_pointcloud'])

    def execute(self, userdata):

        pcl = AttributedPointCloud()
        sample_size = 200
        # Define accumulation points
        hp = []
        hp.append([16.0, 16.0, 0.98])
        hp.append([25.0, 11.0, 1.32])
        # hp.append([18.6, 7.3, 1.80])
        hp.append([1.916, 22.485, 1.0])

        object_type_names = ['Smacks'] #more?
        for idx in xrange(len(hp)):
            for cnt in xrange(sample_size):
                random_vector = self.get_random_vector(hp[idx], [.5, .5, 0.2])
                random_quaternion = self.get_random_quaternion() 

                element = AttributedPoint()
                pose = Pose()
                pose.orientation = Quaternion(*random_quaternion)
                pose.position = Point(*random_vector)
                element.type = 'Smacks'
                element.pose = pose
                pcl.elements.append(element)

        userdata.object_pointcloud = pcl
        return 'succeeded'

