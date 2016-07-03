#!/usr/bin/env python
#
#  A Python package for temporal consistency and scheduling.
#
#  Copyright (c) 2015 MIT. All rights reserved.
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
pytemporal: A Python package for temporal consistency and scheduling.

Miscellaneous utilities related to temporal plans.

@author: Pedro Santana (psantana@mit.edu).
"""
import xml.etree.cElementTree as ET
import json
import time
import pickle
from rmpyl.rmpyl import RMPyL
from rmpyl.defs import Event
from rmpyl.constraints import TemporalConstraint

def print_schedule(schedule,sort_by_time=True):
    """
    Prints the time assignments (schedules) to temporal events, with the possibility
    of sorting them by time.
    """
    sched_tuples = [(e,t) for e,t in schedule.items()]
    if sort_by_time:
        sched_tuples.sort(key=lambda x: x[1])
    for e,t in sched_tuples:
        print('\t%s: %.2f s'%(e,t))

def print_squeezings(squeeze_dict):
    """
    Prints the squeezings of temporal constraints.
    """
    for tc,tc_dict in squeeze_dict.items():
        print('%s: [%.1f,%.1f] -> [%.1f,%.1f], Risk=%f%%'%(tc.name,tc.lb,tc.ub,
                                                              tc_dict['lb'],tc_dict['ub'],
                                                              tc_dict['risk']*100.0))


def to_rmpyl(tcs):
    """
    Converts the temporal constraints into an RMPyL program, which can then
    be exported to a TPN.
    """
    prog = RMPyL()
    for tc in tcs:
        prog.add_temporal_constraint(tc)
    return prog

def from_rmpyl(prog):
    """
    Imports the temporal constraints from an RMPyL program.
    """
    if len(prog.choices)==0:
        tcs = [e.duration for e in prog.primitive_episodes]
        tcs+= [t for t in prog.temporal_constraints]
        return tcs
    else:
        raise NotImplementedError('PyTemporal cannot deal with disjuntive constraints at the moment.')

def write_pstnu_benchmark(instance_list,bench_name,filename):
    """
    Writes a dictionary of benchmark PSTNU problems to disk
    """
    with open(filename,'w') as f:
        json.dump({'name':bench_name,'instances':instance_list},f,indent=1,separators=(',', ': '))

def rmpyl_pstnu_to_dict(rmpyl_pstnu,name):
    """
    Converts a PSTNU represented as a list of RMPyL constraints into a standardized
    dictionary representation.
    """
    return {name:[rmpyl_tc_to_dict(tc) for tc in rmpyl_pstnu]}

def rmpyl_tc_to_dict(rmpyl_tc):
    """
    Converts an RMPyL temporal constraint into dictionary form.
    """
    tc_dict={'name':rmpyl_tc.name,
             'start_event_name':rmpyl_tc.start.name,
             'end_event_name':rmpyl_tc.end.name,
             'type':rmpyl_tc.type}
    if rmpyl_tc.type in ['controllable','uncontrollable_bounded']:
        tc_dict['properties']={'lb':rmpyl_tc.lb,'ub':rmpyl_tc.ub}
    else:
        tc_dict['properties']={'distribution':rmpyl_tc.distribution}
    return tc_dict

def load_pstnu_benchmark(filename):
    """
    Returns a list of RMPyL PSTNU instances from a JSON file.
    """
    with open(filename,'r') as f:
        bench_dict = json.load(f)
    return bench_dict

def from_json_to_rmpyl_benchmark(bench_json):
    """
    Reads in a JSON benchmark file and outputs an RMPyL benchmark dictionary.
    """
    print('Loading JSON tests.')
    start_time=time.time()
    bench_dict = load_pstnu_benchmark(bench_json)
    print('Loading took %.3f seconds'%(time.time()-start_time))

    print('Converting to RMPyL')
    benchmark={}
    start_time=time.time()
    for inst in bench_dict['instances']:
        for test_name, tc_dict_list in inst.items():
            benchmark[test_name]=dict_to_rmpyl_pstnu(tc_dict_list)
    print('Conversion took took %.3f seconds'%(time.time()-start_time))
    return benchmark

def dict_to_rmpyl_pstnu(tc_dict_list):
    """
    Converts a PSTNU instance in dictionary form to its corresponding
    representation as a list of RMPyL constraints.
    """
    tcs=[]
    event_dict = _get_event_dictionary(tc_dict_list)
    for tc_dict in tc_dict_list:
        param_dict={'name':tc_dict['name'],
                    'start':event_dict[tc_dict['start_event_name']],
                    'end':event_dict[tc_dict['end_event_name']]}
        if tc_dict['type'].lower() in ['controllable','uncontrollable_bounded','uncontrollable_probabilistic']:
            param_dict['ctype'] = tc_dict['type'].lower()
        else:
            raise ValueError('%s is not a valid type of temporal constraint'%tc_dict['type'])

        if param_dict['ctype'] in ['controllable','uncontrollable_bounded']:
            param_dict['lb'] = tc_dict['properties']['lb']
            param_dict['ub'] = tc_dict['properties']['ub']
        else:
            param_dict['distribution']=tc_dict['properties']['distribution']

        tcs.append(TemporalConstraint(**param_dict))
    return tcs

def from_cctp_pstp(cctp_file):
    """
    Converts a pSTP in CCTP format into RMPyL temporal constraints.
    """
    tree = ET.parse(cctp_file)
    cctp = tree.getroot()

    event_dict=_cctp_events(cctp)
    return [_arc_to_temporal_constraint(arc,event_dict) for arc in cctp.findall('ARC')]

def load_pickle_file(pickle_file):
    print('Loading '+pickle_file)
    start_time=time.time()
    with open(pickle_file,'rb') as f:
        results = pickle.load(f)
    print('Loading took %.3f seconds'%(time.time()-start_time))
    return results

def _get_event_dictionary(tc_dict_list):
    """
    Returs mapping from event names to event objects.
    """
    event_dict={}
    for tc in tc_dict_list:
        for e_name in (tc['start_event_name'],tc['end_event_name']):
            if not (e_name in event_dict):
                event_dict[e_name] = Event(name=e_name)
    return event_dict

def _cctp_events(cctp_root):
    """
    Returns a dictionary from CCTP event ID's to event objects.
    """
    event_dict={}
    for event in cctp_root.findall('EVENT'):
        event_dict[event.find('ID').text]=Event()
    return event_dict

def _arc_to_temporal_constraint(cctp_arc,event_dict):
    """
    Converts a CCTP arc XML element into a temporal constraint
    """
    typ=cctp_arc.find('TYPE').text
    start_event=event_dict[cctp_arc.find('START').text]
    end_event=event_dict[cctp_arc.find('END').text]

    if typ=='Controllable':
        lb=float(cctp_arc.find('LOWERBOUND').text)
        ub=float(cctp_arc.find('UPPERBOUND').text)
        return TemporalConstraint(start=start_event,end=end_event,ctype='controllable',
                                  lb=lb,ub=ub)
    elif typ=='Uncontrollable':
        distribution=cctp_arc.find('DISTRIBUTION').text
        if distribution=='Normal':
            mean=float(cctp_arc.find('MEAN').text)
            var=float(cctp_arc.find('VAR').text)
            return TemporalConstraint(start=start_event,end=end_event,
                                      ctype='uncontrollable_probabilistic',
                                      distribution={'type':'gaussian','mean':mean,
                                                    'variance':var})
        elif distribution=='Uniform':
            lb=float(cctp_arc.find('LOWERBOUND').text)
            ub=float(cctp_arc.find('UPPERBOUND').text)
            return TemporalConstraint(start=start_event,end=end_event,
                                      ctype='uncontrollable_probabilistic',
                                      distribution={'type':'uniform','lb':lb,'ub':ub})
        else:
            raise ValueError('Unknown type of CCTP distribution: '+distribution)
    else:
        raise ValueError('Unknown type of CCTP arc: '+typ)
