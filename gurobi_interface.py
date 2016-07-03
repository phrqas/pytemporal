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

Interface functions with the Gurobi solver, which is used for temporal consistency
and temporal relaxations.

@author: Pedro Santana (psantana@mit.edu).
"""
from gurobipy import *
#from . import gurobi_sc as sc
from . import paris as sc

class GurobiInterface(object):
    """
    Interface with the Gurobi solver for tasks related to temporal problems.
    """
    def __init__(self):
        self.model =  Model('gurobi_temporal')
        self.model.setParam('OutputFlag',0) #Suppresses terminal output
        self._var_map={}
        self._tc_map={}

        #Returns the minimum risk pSTN with a strongly controllable STNU reformulation.
        #self.strongly_controllable_stp = sc.strongly_controllable_stp

    def add_temporal_constraint(self,tc):
        """
        Adds a temporal constraint to the Gurobi model representing the scheduling
        problem.
        """
        if not tc.id in self._tc_map: #Does not add a constraint twice
            #Add the event variables to the model
            start_var,end_var = self._add_event_variables(tc,self.model,self._var_map)
            # Add upper and lower bound linear constraints
            lb_const = self.model.addConstr(end_var - start_var >= tc.lb)
            ub_const = self.model.addConstr(end_var - start_var <= tc.ub)

            self._tc_map[tc.id] = [lb_const,ub_const]

    def remove_temporal_constraint(self,tc):
        """
        Removes a temporal constraint from the Gurobi model representing the scheduling
        problem.
        """
        if tc.id in self._tc_map:
            for ctr in self._tc_map[tc.id]:
                self.model.remove(ctr)
            del self._tc_map[tc.id]

    def consistent_stn(self,add_tcs=[]):
        """
        Returns whether a Simple Temporal Network is consistent.
        """
        if len(add_tcs)>0:
            for tc in add_tcs: #Adds temporary constraints
                self.add_temporal_constraint(tc)

            self.model.optimize()#Checks feasibility only (no objective)
            infeasible = self.model.status==GRB.status.INFEASIBLE

            for tc in add_tcs: #Removes temporary constraints
                self.remove_temporal_constraint(tc)

            return not infeasible
        else:
            self.model.optimize() #Checks feasibility only (no objective)
            return not self.model.status==GRB.status.INFEASIBLE

    def consistent_stn_incremental(self,add_tcs=[]):
        """
        Incremental version of the consistent_stn function, that reuses a
        previous solution object to give Gurobi a warm start.
        """
        raise NotImplementedError('Incremental consistency not implemented for Gurobi yet.')

    def stn_relaxation(self,tcs=None,weight_dict={}):
        """
        Returns the minimum cost relaxation of an STN to restore feasibility.
        """
        relax_model = Model('relaxation_model')
        relax_model.setParam('OutputFlag',0) #No output to console

        relax_model.modelSense = GRB.MINIMIZE #Objective is to minimize relaxation cost

        var_map={}; tc_relax_var_dict={}
        for tc in tcs:
            #Adds event variables to relaxation model
            start_var,end_var = self._add_event_variables(tc,relax_model,var_map)

            if len(weight_dict)==0:
                lb_relax_cost=1.0; ub_relax_cost=1.0
            else:
                if tc in weight_dict:
                    lb_relax_cost=weight_dict[tc][0]; ub_relax_cost=weight_dict[tc][1]
                else:
                    lb_relax_cost=0.0; ub_relax_cost=0.0

            lb_var = relax_model.addVar(vtype=GRB.CONTINUOUS,lb=0.0,obj=lb_relax_cost,
                                  name='lb_'+tc.name)
            ub_var = relax_model.addVar(vtype=GRB.CONTINUOUS,lb=0.0,obj=ub_relax_cost,
                                  name='ub_'+tc.name)

            relax_model.update() # Integrate new variables

            #Stores a reference from the temporal constraint to its relaxation
            #variables
            tc_relax_var_dict[tc]=[lb_var,ub_var]

            #Adds temporal constraints with relaxation variables
            lb_const = relax_model.addConstr(end_var-start_var+lb_var>=tc.lb,'lb_'+tc.name)
            ub_const = relax_model.addConstr(end_var-start_var-ub_var<=tc.ub,'ub_'+tc.name)

        relax_model.optimize()#Performs minimum cost relaxation

        if relax_model.status == GRB.status.OPTIMAL: #Found optimal relaxation
            #Returns amount by which temporal constraints should be relaxed,
            #but only for the ones with non-zero (or very close to zero)
            #relaxations
            relax_dict={}
            for tc,(lb_var,ub_var) in tc_relax_var_dict.items():
                if (lb_var.x>1e-5)or(ub_var.x>1e-5):
                    relax_dict[tc]=[lb_var.x,ub_var.x]
            return relax_dict,relax_model.objVal
        else: #No relaxation found
            return None,None

    def _add_event_variables(self,tc,model,var_map):
        """
        Enforces the correspondence between events and Gurobi variables.
        """
        update_model=False
        if tc.start.id in var_map:
            start_var = var_map[tc.start.id]
        else:
            start_var = model.addVar(vtype=GRB.CONTINUOUS,lb=0.0,name=tc.start.name)
            var_map[tc.start.id] = start_var
            update_model = True

        if tc.end.id in var_map:
            end_var = var_map[tc.end.id]
        else:
            end_var = model.addVar(vtype=GRB.CONTINUOUS,lb=0.0,name=tc.end.name)
            var_map[tc.end.id] = end_var
            update_model = True

        # Integrate new variables, if necessary
        if update_model:
            model.update()

        return start_var,end_var
