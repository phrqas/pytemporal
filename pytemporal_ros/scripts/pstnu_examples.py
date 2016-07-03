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
PSTNU examples useful for testing PARIS

@author: Pedro Santana (psantana@mit.edu).
"""
from rmpyl.defs import Event
from pytemporal.interface import TemporalConstraint

def stnu_stedl():
    """
    Example Simple Temporal Problem in John Stedl's thesis.
    """
    events = [Event(name='s1'),Event(name='s2'),Event(name='e1'),Event(name='e2')]
    tcs = [TemporalConstraint(start=events[0],end=events[2],ctype='uncontrollable_bounded',
                              lb=5.0,ub=10.0),
           TemporalConstraint(start=events[0],end=events[1],ctype='controllable',
                              lb=4.0,ub=float('inf')),
           TemporalConstraint(start=events[1],end=events[3],ctype='uncontrollable_bounded',
                              lb=1.0,ub=2.0),
           TemporalConstraint(start=events[3],end=events[2],ctype='controllable',
                              lb=0.0,ub=float('inf'))]
    return tcs

def stp_uniform_rubato():
    """
    Strong controllability example from the Rubato paper using uniform uncertainty,
    instead of uniform.
    """
    events = [Event(name='s%d'%(i)) for i in range(8)]
    tcs = [TemporalConstraint(start=events[0],end=events[1], #tear-down A
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'uniform','lb':9.0,'ub':21.0}),
           TemporalConstraint(start=events[0],end=events[2], #tear-down B
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'uniform','lb':15.0,'ub':45.0}),
           TemporalConstraint(start=events[1],end=events[3], #vacuum A
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'uniform','lb':1.0,'ub':19.0}),
           TemporalConstraint(start=events[3],end=events[5], #set-up A
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'uniform','lb':4.0,'ub':16.0}),
           TemporalConstraint(start=events[4],end=events[6], #vacuum B
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'uniform','lb':5.0,'ub':35.0}),
           TemporalConstraint(start=events[6],end=events[7], #set-up B
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'uniform','lb':4.0,'ub':16.0}),
           TemporalConstraint(start=events[2],end=events[4],ctype='controllable',lb=0.0,ub=float('inf')),#wait
           TemporalConstraint(start=events[3],end=events[4],ctype='controllable',lb=0.0,ub=float('inf')),#wait
           TemporalConstraint(start=events[0],end=events[5],ctype='controllable',lb=0.0,ub=60.0), #set-up A in 1 hour
           TemporalConstraint(start=events[0],end=events[7],ctype='controllable',lb=0.0,ub=90.0)] #set-up B in 1.5 hours

    return tcs

def stp_rubato():
    """
    Strong controllability example from the Rubato paper using uniform uncertainty,
    instead of uniform.
    """
    events = [Event(name='start'),Event(name='end-tear-down-A'),
              Event(name='end-tear-down-B'),Event(name='end-vaccum-A'),
              Event(name='start-vaccum-B'),Event(name='end-set-up-A'),
              Event(name='end-vaccum-B'),Event(name='end-set-up-B')]

    tcs = [TemporalConstraint(start=events[0],end=events[1],
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'gaussian','mean':15.0,'variance':2},
                              name='tear-down-A'),
           TemporalConstraint(start=events[0],end=events[2],
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'gaussian','mean':30.0,'variance':5.0},
                              name='tear-down-B'),
           TemporalConstraint(start=events[1],end=events[3],
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'gaussian','mean':10.0,'variance':3.0},
                              name='vacuum-A'),
           TemporalConstraint(start=events[3],end=events[5],
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'gaussian','mean':10.0,'variance':2.0},
                              name='set-up-A'),
           TemporalConstraint(start=events[4],end=events[6],
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'gaussian','mean':20.0,'variance':5.0},
                              name='vacuum-B'),
           TemporalConstraint(start=events[6],end=events[7],
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'gaussian','mean':10.0,'variance':2.0},
                              name='set-up-B'),
           TemporalConstraint(start=events[2],end=events[4],ctype='controllable',
                              lb=0.0,ub=float('inf'),name='wait1'),
           TemporalConstraint(start=events[3],end=events[4],ctype='controllable',
                              lb=0.0,ub=float('inf'),name='wait2'),
           TemporalConstraint(start=events[0],end=events[5],ctype='controllable',
                              lb=0.0,ub=60.0,name='set-up-A-in-1-hour'),
           TemporalConstraint(start=events[0],end=events[7],ctype='controllable',
                              lb=0.0,ub=90.0,name='set-up-A-in-1.5-hours')]

    return tcs

def stp_picard():
    """
    Strong controllability example from the Picard paper.
    """
    events = [Event(name='dep'), Event(name='arr'),
              Event(name='SoD'),Event(name='erupt')]

    tcs = [TemporalConstraint(start=events[0],end=events[1],
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'gaussian','mean':20.0,'variance':4.0},
                              name='traversal'),
           TemporalConstraint(start=events[2],end=events[3],
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'gaussian','mean':60.0,'variance':25.0},
                              name='eruption'),
           TemporalConstraint(start=events[3],end=events[1],ctype='controllable',
                              lb=0.0,ub=120.0,name='arrival')]

    return tcs

def rover_coordination_example():
    """
    Strong controllability example from the ICAPS'16 paper using proabilistic
    uncertainty (gaussian), set-bounded and controllable events.
    """
    events = [Event(name='start'),
              Event(name='start-opportunity-go-gully'),Event(name='end-opportunity-go-gully'),
              Event(name='start-opportunity-drill'),Event(name='end-opportunity-drill'),
              Event(name='start-opportunity-collect'),Event(name='end-opportunity-collect'),
              Event(name='start-opportunity-process'),Event(name='end-opportunity-process'),
              Event(name='start-opportunity-go-gully-relay'),Event(name='end-opportunity-go-gully-relay'),
              Event(name='start-opportunity-relay'),Event(name='end-opportunity-relay'),

              Event(name='start-spirit-go-rock-outcrop'),Event(name='end-spirit-go-rock-outcrop'),
              Event(name='start-spirit-drill-rock-outcrop'),Event(name='end-spirit-drill-rock-outcrop'),
              Event(name='start-spirit-collect-rock-outcrop'),Event(name='end-spirit-collect-rock-outcrop'),
              Event(name='start-spirit-process-rock-outcrop'),Event(name='end-spirit-process-rock-outcrop'),
              Event(name='start-spirit-go-rock-outcrop-slope-streak'),Event(name='end-spirit-go-rock-outcrop-slope-streak'),
              Event(name='start-spirit-drill-slope-streak'),Event(name='end-spirit-drill-slope-streak'),
              Event(name='start-spirit-collect-slope-streakp'),Event(name='end-spirit-collect-slope-streak'),
              Event(name='start-spirit-process-slope-streak'),Event(name='end-spirit-process-slope-streak'),
              Event(name='start-spirit-go-slope-streak-relay'),Event(name='end-spirit-go-slope-streak-relay'),
              Event(name='start-spirit-relay'),Event(name='end-spirit-relay')]

    tcs = [TemporalConstraint(start=events[0],end=events[1],
                              ctype='controllable',
                              lb=0.0,ub=float('inf'),
                              name='start-opportunity'),
           TemporalConstraint(start=events[1],end=events[2],
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'gaussian','mean':110.0,'variance':20.0},
                              name='opportunity-go-to-gully'),
           TemporalConstraint(start=events[2],end=events[3],
                              ctype='controllable',
                              lb=0.0,ub=float('inf'),
                              name='opportunity-get-ready-for-drill'),
           TemporalConstraint(start=events[3],end=events[4],
                              ctype='uncontrollable_bounded',
                              lb=9.0,ub=10.0,
                              name='opportunity-drill-rock'),
           TemporalConstraint(start=events[4],end=events[5],
                              ctype='controllable',
                              lb=0.0,ub=float('inf'),
                              name='opportunity-get-ready-for-sample'),
           TemporalConstraint(start=events[5],end=events[6],
                              ctype='uncontrollable_bounded',
                              lb=10.0,ub=15.0,
                              name='opportunity-collect-sample'),
           TemporalConstraint(start=events[6],end=events[7],
                              ctype='controllable',
                              lb=0.0,ub=float('inf'),
                              name='opportunity-get-ready-for-process'),
           TemporalConstraint(start=events[7],end=events[8],
                              ctype='uncontrollable_bounded',
                              lb=5.0,ub=30.0,
                              name='opportunity-process-sample'),
           TemporalConstraint(start=events[8],end=events[9],
                              ctype='controllable',
                              lb=0.0,ub=float('inf'),
                              name='opportunity-get-ready-to-go-to-relay'),
           TemporalConstraint(start=events[9],end=events[10],
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'gaussian','mean':150.0,'variance':25.0},
                              name='opportunity-go-from-gully-to-relay'),
           TemporalConstraint(start=events[10],end=events[11],
                              ctype='controllable',
                              lb=0.0,ub=float('inf'),
                              name='opportunity-get-ready-to-relay'),
           TemporalConstraint(start=events[11],end=events[12],
                              ctype='controllable',
                              lb=20.0,ub=30.0, #PHS
                              name='opportunity-relay'),

           TemporalConstraint(start=events[0],end=events[13],
                              ctype='controllable',
                              lb=0.0,ub=float('inf'),
                              name='start-spirit'),
           TemporalConstraint(start=events[13],end=events[14],
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'gaussian','mean':230.0,'variance':40.0},
                              name='spirit-go-to-rock-outcrop'),
           TemporalConstraint(start=events[14],end=events[15],
                              ctype='controllable',
                              lb=0.0,ub=float('inf'),
                              name='spirit-get-ready-for-drill-rock-outcrop'),
           TemporalConstraint(start=events[15],end=events[16],
                              ctype='uncontrollable_bounded',
                              lb=9.0,ub=10.0,
                              name='spirit-drill-rock-outcrop'),
           TemporalConstraint(start=events[16],end=events[17],
                              ctype='controllable',
                              lb=0.0,ub=float('inf'),
                              name='spirit-get-ready-for-sample-rock-outcrop'),
           TemporalConstraint(start=events[17],end=events[18],
                              ctype='uncontrollable_bounded',
                              lb=10.0,ub=15.0,
                              name='spirit-collect-sample-rock-outcrop'),
           TemporalConstraint(start=events[18],end=events[19],
                              ctype='controllable',
                              lb=0.0,ub=float('inf'),
                              name='spirit-get-ready-for-process-rock-outcrop'),
           TemporalConstraint(start=events[19],end=events[20],
                              ctype='uncontrollable_bounded',
                              lb=5.0,ub=30.0,
                              name='spirit-process-sample-rock-outcrop'),
           TemporalConstraint(start=events[20],end=events[21],
                              ctype='controllable',
                              lb=0.0,ub=float('inf'),
                              name='spirit-get-ready-to-go-to-slope-streak'),
           TemporalConstraint(start=events[21],end=events[22],
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'gaussian','mean':180.0,'variance':30.0},
                              name='spirit-go-from-rock-outcrop-to-slope-streak'),
           TemporalConstraint(start=events[22],end=events[23],
                              ctype='controllable',
                              lb=0.0,ub=float('inf'),
                              name='spirit-get-ready-for-drill-slope-streak'),
           TemporalConstraint(start=events[23],end=events[24],
                              ctype='uncontrollable_bounded',
                              lb=9.0,ub=10.0,
                              name='spirit-drill-slope-streak'),
           TemporalConstraint(start=events[24],end=events[25],
                              ctype='controllable',
                              lb=0.0,ub=float('inf'),
                              name='spirit-get-ready-for-sample-slope-streak'),
           TemporalConstraint(start=events[25],end=events[26],
                              ctype='uncontrollable_bounded',
                              lb=9.0,ub=15.0,
                              name='spirit-collect-sample-slope-streak'),
           TemporalConstraint(start=events[26],end=events[27],
                              ctype='controllable',
                              lb=0.0,ub=float('inf'),
                              name='spirit-get-ready-for-process-slope-streak'),
           TemporalConstraint(start=events[27],end=events[28],
                              ctype='uncontrollable_bounded',
                              lb=5.0,ub=30.0,
                              name='spirit-process-sample-slope-streak'),
           TemporalConstraint(start=events[28],end=events[29],
                              ctype='controllable',
                              lb=0.0,ub=float('inf'),
                              name='spirit-get-ready-to-go-to-relay'),
           TemporalConstraint(start=events[29],end=events[30],
                              ctype='uncontrollable_probabilistic',
                              distribution={'type':'gaussian','mean':90.0,'variance':15.0},
                              name='spirit-go-from-slope-streak-to-relay'),
           TemporalConstraint(start=events[30],end=events[31],
                              ctype='controllable',
                              lb=0.0,ub=float('inf'),
                              name='spirit-get-ready-to-relay'),
           TemporalConstraint(start=events[31],end=events[32],
                              ctype='controllable',
                              lb=30.0,ub=40.0, #PHS
                              name='spirit-relay'),

           TemporalConstraint(start=events[32],end=events[11],
                              ctype='controllable',
                              lb=0.0,ub=10.0,
                              name='spirit-relay-before-opportunity'),

           TemporalConstraint(start=events[0],end=events[31],
                              ctype='controllable',
                              lb=600,ub=float('inf'), #PHS
                              name='orbiter-trasmission-start_time-window'),
           TemporalConstraint(start=events[0],end=events[12],
                              ctype='controllable',
                              lb=0.0,ub=700,   #PHS
                              name='orbiter-trasmission-end-time-window')]

    return tcs
