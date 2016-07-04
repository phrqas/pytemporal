#!/usr/bin/env python
#
#  A Python package for temporal consistency and scheduling.
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
pytemporal: A Python package for temporal consistency and scheduling.

Strong consistency checker for PTPN's, as presented in (Santana & Williams, ICAPS14)

@author: Pedro Santana (psantana@mit.edu).
"""
from operator import mul
import itertools
from rmpyl.defs import ChoiceAssignment
from paris import PARIS
import ipdb

class PTPNStrongConsistency(object):
    """
    Strong consistency checker for PTPN's (Santana & Williams, ICAPS14) that caches
    previous solutions for efficiency
    """
    def __init__(self,paris_params={},verbose=False):
        #Uses PARIS as a risk-aware scheduler for PSTNU's.
        self._paris = PARIS(**paris_params)

        #Mapping from sets of assignments to controllable choices to scheduling risk
        self._sol_cache = {}

        self.verbose=verbose #Verbosity flag

    def strong_consistency(self,assignments,tcs):
        """
        Solves the strong consistency problem, should the solution not be in the
        cache.
        """
        decisions = frozenset([a for a in assignments if a.var.type=='controllable'])
        # decision_tuples = frozenset([(a.var,a.value) for a in decisions])
        #
        # add_tcs_tuples = frozenset([(tc.start,tc.end) for tc in tcs])
        #
        # if not ((decision_tuples,add_tcs_tuples) in self._sol_cache):
        #     self._sol_cache[(decision_tuples,add_tcs_tuples)] = self._strong_consistency(decisions,tcs)
        # return self._sol_cache[(decision_tuples,add_tcs_tuples)]
        return self._strong_consistency(decisions,tcs)

    def _strong_consistency(self,decisions,tcs):
        """
        Determines if a set of assignments to controllable choices can give
        rise to a strongly consistent PTPN schedule with respect to the
        uncontrollable choices.
        """
        tcs_with_active_decisions = self._tcs_with_active_decisions(decisions,tcs)
        return self._consistent_scenario_prob_mass(tcs_with_active_decisions,decisions)

    def _tcs_with_active_decisions(self,decisions,tcs):
        """
        Temporal constraints whose supports are consistent with the decisions
        and depend only on assignments to uncontrollable choices. Returns a
        dictionary mapping the temporal to the uncontrollable choices that
        potentially activate them.
        """
        #Controllable choices in the assignment
        decision_vars = set([a.var for a in decisions])

        #All temporal constraints whose supports are consistent with the
        #decisions and depend only on assignments to uncontrollable choices, along
        #with said uncontrollable choices.
        tcs_with_active_decisions = {}

        for tc in tcs:
            if tc.is_consistent(decisions):
                for set_idx,decision_set in enumerate(tc.support_decisions):
                    #All controllable choices (decisions) in this constraint's
                    #support have been assigned
                    if decision_set <= decision_vars:
                        #Maps the temporal constraint to the set of uncontrollable
                        #choices (observations) that might activate it.
                        tcs_with_active_decisions[tc]=tc.support_observations[set_idx]
                        break

        return tcs_with_active_decisions

    def _consistent_scenario_prob_mass(self,tcs_with_active_decisions,decisions):
        """
        Determines the scenarios (full assignments to uncontrolalble)
        """
        #TODO:I need to assign observations according to their hierarchy, as opposed
        #to assigning them independently of the choice structure.
        #
        observation_list = list(itertools.chain(*tcs_with_active_decisions.values()))
        init_assignments = set(decisions)
        init_active,init_consistent = self._split_consistent_constraints(tcs_with_active_decisions.keys(),
                                                                         init_assignments)
        queue=[{'assignments':init_assignments,
                'active_tcs':init_active,
                'consistent_tcs':init_consistent}]

        prob_success_mass=0.0

        # ipdb.set_trace()
        while len(queue)>0:
            node = queue.pop() #Removes element from the queue

            #Checks if they are strongly controllable with PARIS
            squeeze_dict,risk_bound,sc_schedule = self._paris.schedule(node['active_tcs'])

            #Jointly active constraints are temporally consistent
            if risk_bound != None:
                #Still constraints left to activate
                if len(node['consistent_tcs'])>0:

                    #All variables assigned so far
                    assigned_vars = set([assig.var for assig in node['assignments']])
                    min_index = len(observation_list)
                    #Finds the index of thefirst observation variable in the
                    #observation list which has not been assigned.
                    for tc in node['consistent_tcs']:
                        unassigned_vars = [o for o in tcs_with_active_decisions[tc] if not o in assigned_vars]
                        min_index = min(min_index,min([observation_list.index(v) for v in unassigned_vars]))

                    next_obs = observation_list[min_index]

                    for val in next_obs.domain:
                        new_assig = node['assignments'] | {ChoiceAssignment(next_obs,val,False)}
                        new_active,new_consistent = self._split_consistent_constraints(node['consistent_tcs'],new_assig)

                        queue.append({'assignments':new_assig,
                                      'active_tcs':node['active_tcs']+new_active,
                                      'consistent_tcs':new_consistent})

                #All variables have been assigned, so it is a model
                else:
                    #Assumes uncontrollable choices are independent, so their
                    #joint probability is just the multiplication
                    prob_choices = [a.probability for a in node['assignments'] if a.var.type=='probabilistic']
                    prob_scenario = reduce(mul,prob_choices,1.0)

                    #Success is the joint probability of the scenario occurring
                    #(first term), multiplied by the probability of the schedule
                    #working (second term). Assumes that random variables for
                    #observations and durations and independent.
                    prob_success_mass += prob_scenario*(1.0-risk_bound)

        return prob_success_mass

    def _split_consistent_constraints(self,tcs,assignments):
        """
        Splits temporal constraints that are active; those that are not active,
        but consistent; and those that are inconsistent.
        """
        active=[];consistent=[]
        for tc in tcs:
            if tc.is_active(assignments):
                active.append(tc)
            elif tc.is_consistent(assignments):
                consistent.append(tc)
        return active,consistent
