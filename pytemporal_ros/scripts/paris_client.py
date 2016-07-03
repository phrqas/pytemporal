#!/usr/bin/env python
#
#  Copyright (c) 2016 MIT. All rights reserved.
#
#   author: Pedro Santana
#   e-mail: psantana@mit.edu
#   website: people.csail.mit.edu/psantana
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#  1. Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#  2. Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in
#     the documentation and/or other materials provided with the
#     distribution.
#  3. Neither the name(s) of the copyright holders nor the names of its
#     contributors or of the Massachusetts Institute of Technology may be
#     used to endorse or promote products derived from this software
#     without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
#  OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
#  AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
"""
Example Python node that interacts with the PARIS server.

@author: Pedro Santana (psantana@mit.edu).
"""
import rospy
from pytemporal_ros.srv import PARISSrv,PARISSrvRequest
import utils as ut
import pstnu_examples as ex
import sys
import time

class PARISClient(object):
    """Class encapsulating the PARIS server functionality."""
    def __init__(self):
        rospy.loginfo('Waiting for PARIS service to be available')
        rospy.wait_for_service('paris_service')
        self._paris_srv = rospy.ServiceProxy('paris_service', PARISSrv)
        rospy.loginfo('PARIS service found!')

    def request_service(self,tcs,gaussian_div=5,gaussian_optimize_partition=False,
                 gaussian_lr=0.05,gaussian_tol=1e-5,gaussian_max_iter=10000,cc=-1.0,
                 makespan=False):
        """Request a schedule from PARIS."""
        start_time = time.time()
        req = PARISSrvRequest()

        #PSTNU
        req.pstnu = ut.rmpyl_temporal_constraints_to_ros_pstnu(tcs)
        #Number of divisions on each side of a Gaussian
        req.gaussian_div = gaussian_div
        #Whether to optimize Gaussian partitions or not
        req.gaussian_optimize_partition = gaussian_optimize_partition
        #Learning rate for gradient descent
        req.gaussian_lr = gaussian_lr
        #Error tolerance for gradient descent
        req.gaussian_tol = gaussian_tol
        #Maximum number of iterations for gradient descent
        req.gaussian_max_iter = gaussian_max_iter
        #Chance constraint for the strong schedule
        req.cc=cc
        #Whether to optimize makespan (True) or minimize scheduling risk (False)
        req.makespan = makespan

        req.header.stamp = rospy.Time.now()
        resp = self._paris_srv(req)
        elapsed_time = time.time()-start_time

        if resp == None:
            rospy.logwarn('PARIS service failed')
        else:
            rospy.loginfo('PARIS service replied!')
            ut.print_paris_response(resp)

        print('\nTotal time (including ROS overhead): %.4f s\n'%(elapsed_time))
        return resp

if __name__=='__main__':
    rospy.init_node('paris_client_node')
    rospy.loginfo('Initializing PARIS client')
    paris_server = PARISClient()

    pstnu_examples = [ex.stnu_stedl,
                      ex.stp_uniform_rubato,
                      ex.stp_rubato,
                      ex.stp_picard,
                      ex.rover_coordination_example]

    if len(sys.argv)<2:
        print('Please choose an integer option between 0 and 4.')
        sys.exit(0)
    else:
        option = int(sys.argv[1])
        example_gen = pstnu_examples[option]
        paris_server.request_service(example_gen())

    rospy.loginfo('Press Ctrl+C to terminate.')
    rospy.spin()
    rospy.loginfo('PARIS client terminated')
