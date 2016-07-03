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
ROS node that allows PARIS to be called as a service.

@author: Pedro Santana (psantana@mit.edu).
"""
import rospy
from pytemporal_ros.srv import PARISSrv,PARISSrvResponse
from pytemporal.paris import PARIS
import utils as ut

class PARISServer(object):
    """Class encapsulating the PARIS server functionality."""
    def __init__(self):
        rospy.loginfo('Creating PARIS service')
        self._paris_service = rospy.Service('paris_service', PARISSrv,self._paris_service_cb)
        rospy.loginfo('PARIS service created')

    def _paris_service_cb(self,req):
        """Handler of PARIS service requests."""
        resp = PARISSrvResponse()
        tcs = ut.ros_pstnu_to_rmpyl_temporal_constraints(req.pstnu)
        paris = PARIS(req.gaussian_div,req.gaussian_optimize_partition,
                      req.gaussian_lr,req.gaussian_tol,req.gaussian_max_iter)

        if paris.set_minimum_risk_strong_controllability(tcs):
            setup_success = True
            if req.makespan:
                rospy.loginfo('Setting up makespan optimization.')
                paris.set_makespan_optimization()
            if req.cc>=0.0:
                rospy.loginfo('Setting up chance-constrained optimization.')
                paris.set_chance_constraint(req.cc)
            rospy.loginfo('Calling PARIS to solve scheduling problem.')
            squeeze_dict,risk_bound,sc_schedule = paris.solve()
        else:
            setup_success = False
            squeeze_dict=None

        if not setup_success:
            rospy.logwarn('Failed to set up strong controllability problem.')

        if squeeze_dict != None:
            resp.success = True
            resp.schedule = ut.schedule_to_schedule_msg(sc_schedule)
            resp.squeezes = ut.squeeze_dict_to_squeeze_msgs(squeeze_dict)
            resp.risk_upper_bound = risk_bound
            rospy.loginfo('Strong schedule found!')
        else:
            resp.success = False
            rospy.logwarn('Failed to solve strong controllability problem.')

        resp.header.stamp = rospy.Time.now() #Records time when solution was obtained
        return resp

if __name__=='__main__':
    rospy.init_node('paris_service_node')
    rospy.loginfo('Initializing the PARIS server')
    paris_server = PARISServer()
    rospy.spin()
    rospy.loginfo('PARIS server terminated')
