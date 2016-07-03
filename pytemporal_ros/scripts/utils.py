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
Tools to facilitate the use of PARIS as a ROS service.

@author: Pedro Santana (psantana@mit.edu).
"""
import pytemporal.interface as pyt
from rmpyl.defs import Event
from pytemporal_ros.msg import PSTNU,StrongSchedule,TemporalConstraint,TemporalConstraintSqueeze

def ros_pstnu_to_rmpyl_temporal_constraints(pstnu):
    """Converts ROS temporal constraints into RMPyL temporal constraints."""
    tcs=[]
    event_dict = _get_event_dictionary(pstnu)
    for tc in pstnu.tcns:
        param_dict={'name':tc.name,
                    'start':event_dict[tc.start_event_name],
                    'end':event_dict[tc.end_event_name]}
        if tc.type.lower() in ['controllable','uncontrollable_bounded','uncontrollable_probabilistic']:
            param_dict['ctype'] = tc.type.lower()
        else:
            raise ValueError('%s is not a valid type of temporal constraint'%tc.type)

        prop_dict = parse_string_properties(tc.properties)

        if param_dict['ctype'] in ['controllable','uncontrollable_bounded']:
            param_dict['lb'] = float(prop_dict['lb'])
            param_dict['ub'] = float(prop_dict['ub'])
        else:
            distrib_dict={}
            for k,v in prop_dict.items():
                dist_key =  k.split('_')[1]
                distrib_dict[dist_key] = v if dist_key=='type' else float(v)
            param_dict['distribution']=distrib_dict

        tcs.append(pyt.TemporalConstraint(**param_dict))

    return tcs

def rmpyl_temporal_constraints_to_ros_pstnu(tcs):
    """Converts RMPyL temporal constraints into ROS temporal constraints."""
    pstnu = PSTNU()

    for tc in tcs:
        tc_ros = TemporalConstraint()
        if tc.type.lower() in ['controllable','uncontrollable_bounded','uncontrollable_probabilistic']:
            tc_ros.type = tc.type.lower()
        else:
            raise ValueError('%s is not a valid type of temporal constraint'%tc.ctype)

        tc_ros.name = tc.name
        tc_ros.start_event_name = tc.start.name
        tc_ros.end_event_name = tc.end.name

        if tc_ros.type in ['controllable','uncontrollable_bounded']:
            tc_ros.properties = ['%s=%s'%(p,str(v)) for p,v in zip(['lb','ub'],[tc.lb,tc.ub])]
        else:
            tc_ros.properties = ['distribution_%s=%s'%(k,str(v)) for k,v in tc.distribution.items()]
        pstnu.tcns.append(tc_ros)

    return pstnu

def squeeze_dict_to_squeeze_msgs(squeeze_dict):
    """Converts a PARIS temporal squeeze dictionary into a temporal squeeze
    message."""
    squeeze_msgs=[]
    for tc,tc_dict in squeeze_dict.items():
        squeeze_msg = TemporalConstraintSqueeze()
        squeeze_msg.name = tc.name
        squeeze_msg.lb_model = tc.lb
        squeeze_msg.ub_model = tc.ub
        squeeze_msg.lb_imposed = tc_dict['lb']
        squeeze_msg.ub_imposed = tc_dict['ub']
        squeeze_msg.squeeze_risk = tc_dict['risk']
        squeeze_msgs.append(squeeze_msg)
    return squeeze_msgs

def schedule_to_schedule_msg(schedule):
    """Converts a PARIS strong schedule into a strong schedule message."""
    schedule_msg = StrongSchedule()
    for ev_name, ev_time in schedule.items():
        schedule_msg.event_names.append(ev_name)
        schedule_msg.event_schedules.append(ev_time)
    return schedule_msg

def parse_string_properties(properties):
    """Returns a dictionary corresponding to a list of properties in the form
    PROPERTY=VALUE"""
    prop_dict = {}
    for p_str in properties:
        k,v = p_str.split('=')
        prop_dict[k]=v
    return prop_dict

def print_paris_response(resp):
    """Prints the PARIS server response to the screen."""
    if resp.success:
        print('\nThis is the schedule:')
        sort_names,sort_scheds=zip(*sorted(zip(resp.schedule.event_names,resp.schedule.event_schedules),key=lambda x:x[1]))
        for ev_name, ev_time in zip(sort_names,sort_scheds):
            print('\t%.2f s:%s%s'%(ev_time,' '*(15-len('%.2f s:'%(ev_time))),ev_name))
        print('\nThese are the squeezings:')
        for tc in resp.squeezes:
            print('\t%s [%.1f,%.1f]->[%.1f,%.1f]:\tRisk=%.4f%%'%(tc.name,
                                                                 tc.lb_model,
                                                                 tc.ub_model,
                                                                 tc.lb_imposed,
                                                                 tc.ub_imposed,
                                                                 tc.squeeze_risk*100.0))
        print('\nRisk bound: %.4f%%'%(resp.risk_upper_bound*100.0))
    else:
        print('\nNo solution found!')

def _get_event_dictionary(pstnu):
    """Returs mapping from event names to event objects."""
    event_dict={}
    for tc in pstnu.tcns:
        for e_name in (tc.start_event_name,tc.end_event_name):
            if not (e_name in event_dict):
                event_dict[e_name] = Event(name=e_name)
    return event_dict
