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

Interface functions with the CPLEX solver, which is used for temporal consistency
and temporal relaxations.

@author: Pedro Santana (psantana@mit.edu).
"""
from cplex import *

class CplexInterface(object):
    """
    Interface with the Cplex solver for tasks related to temporal problems.
    """
    def __init__(self):        
        self.model =  Cplex()        
        self._var_map={}
        self._tc_map={}

    def add_temporal_constraint(self,tc):
        """
        Adds a temporal constraint to the Cplex model representing the scheduling
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

    def stn_relaxation(self,tcs,weight_dict={}):
        """
        Returns the minimum cost relaxation of an STN to restore feasibility.
        """
        relax_model = Model('relaxation_model'); relax_model.setParam('OutputFlag',0)
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



# class CPLEXTemporalChecker(TemporalConsistencyChecker):
#     """Temporal Consistency Checker for STP's (all temporal constraints are
#     controllable) using CPLEX."""    
#     def __init__(self):    
#         super(CPLEXTemporalChecker,self).__init__()
#         self.reset()         

# #    def _event_variables(self,stp,prob):
# #        """sfsd"""
# #        #Adds new event names to the problem
# #        if prob.variables.get_num()>0:
# #            new_n = [n for n in stp.get_event_names() if n not in prob.variables.get_names()] 
# #        else:
# #            new_n = stp.get_event_names()
# #        
# #        lb = [0.0]*len(new_n) #All variables are lower-bounded by 0    
# #        ub = [cplex.infinity]*len(new_n) #All variables are upper-bounded by infinity
# #        
# #        obj = [0.0]*len(new_n)
# #        
# #        return new_n,ub,lb,obj
# #
# #    def _relaxation_variables(self,stp,prob):
# #        """Create two relaxation variables for each temporal constraint
# #        (one for the lower and one for the upper bound)"""
# #        num_constraints = stp.get_num_constraints()
# #        
# #        ub_names = ["dU"+str(i) for i in range(num_constraints)]#upper bounds
# #        lb_names = ["dL"+str(i) for i in range(num_constraints)]#lower bounds                
# #        lb = [0]*2*num_constraints #All lower-bounded by 0        
# #        ub = [cplex.infinity]*2*num_constraints #All upper-bounded by infinity
# #        
# #        obj = [1.0]*2*num_constraints
# #        
# #        return ub_names+lb_names,ub,lb,obj
# #
# #    def _relaxation_constraints(self,stp,prob):
# #        """Represents the simple temporal constraints as a set of linear 
# #        constraints"""
# #        num_constraints = stp.get_num_constraints()
# #
# #        rows = [[] for i in range(2*num_constraints)]
# #        rhs = [[] for i in range(2*num_constraints)]
# #
# #        row_names = ["c"+str(i) for i in range(2*num_constraints)]
# #        senses = "L"*len(row_names)
# #
# #        # lb-dL<=end-start<=ub+dU
# #        #start-end <= dL-lb
# #        for index,stc in enumerate(stp.get_stn()):
# #            #Upper bound constraint
# #            rows[2*index] = [[stc.end_,stc.start_,"dU"+str(index)],[1.0,-1.0,-1.0]]
# #            rhs[2*index] = stc.ub_
# #            #Lower bound constraint
# #            rows[2*index+1] = [[stc.start_,stc.end_,"dL"+str(index)],[1.0,-1.0,-1.0]]
# #            rhs[2*index+1] = -stc.lb_

#     def is_consistent(self,stp,incremental=False):
#         """Checks the feasibility of an STP using CPLEX"""
#         try:
#             #Reuses a previous problem object, should one be passed. Otherwise, creates
#             #a new one.
#             if incremental and (self._prev_problem != None): 
#                 prob = self._prev_problem
#             else: 
#                 prob = cplex.Cplex()
#                 prob.set_results_stream(None) #Suppresses output
#                 prob.set_log_stream(None) #Suppresses output
            
#             #Adds new event names to the problem
#             if prob.variables.get_num()>0:
#                 new_n = [n for n in stp.get_event_names() if n not in prob.variables.get_names()] 
#             else:
#                 new_n = stp.get_event_names()
                
#             lb = [0.0]*len(new_n) #All variables are lower-bounded by 0    
#             ub = [cplex.infinity]*len(new_n) #All variables are upper-bounded by infinity
#             prob.variables.add(names=new_n,lb=lb,ub=ub)    
                
#             #Adds temporal constraints end-start<=ub, start-end <= -lb
#             rows = []; rhs = []    
#             for index,stc in enumerate(stp.get_stn()):
#                 rows.append([[stc.end_, stc.start_],[1.0,-1.0]]) #Upper bound constraint
#                 rhs.append(stc.ub_)                
                
#                 rows.append([[stc.start_,stc.end_],[1.0,-1.0]]) #Lower bound constraint
#                 rhs.append(-stc.lb_)
#             senses=['L']*len(rows)
        
#             #Adds the STCs to the problem
#             prob.linear_constraints.add(lin_expr=rows, senses=senses,rhs=rhs)    
                            
#             self._prev_problem = prob #Stores reference to current problem
#             prob.solve() #Checks feasibility of the temporal constraints
#             if prob.solution.get_status() == prob.solution.status.infeasible:
#                 return False
#             else:
#                 #Stores reference to the current problem
#                 self._prev_problem = prob
#                 return True      
        
#         except CplexError as exc:
#             print(exc)
#             return
    
#     def minimum_relaxation(self,stp,stn_weights):
#         """Find the continuous relaxation of temporal constraints that minimizes
#         the weighted relaxation penalties."""
#         feasible,prob,dUs,dLs = self._stp_continuous_relaxation(stp,stn_weights)
        
#         if feasible:
#             for i,stc in enumerate(stp.get_stn()):
#                 stc.ub_ = stc.ub_+dUs[i]
#                 stc.lb_ = stc.lb_-dLs[i]
        
#         return feasible,prob,dUs,dLs
    
#     def _stp_continuous_relaxation(self,stp,stn_weights):
#         """Frames the minimization problem as an LP and solves it with CPLEX."""
#         try:
#             #Sets as a minimization problem, where the objective is a weighted
#             #sum of continuous constraint relaxations
#             prob = cplex.Cplex()        
#             prob.set_results_stream(None) #Suppresses output
#             prob.set_log_stream(None) #Suppresses output
            
#             prob.objective.set_sense(prob.objective.sense.minimize)
    
#             #Adds the temporal event variables to the problem
#             event_names = stp.get_event_names()
#             num_constraints = stp.get_num_constraints()
#             num_events = len(event_names)
    
#             #For each temporal constraint, we create two relaxation variables
#             #(one for the lower and one for the upper bound)
#             ub_names = ["dU"+str(i) for i in range(num_constraints)]
#             lb_names = ["dL"+str(i) for i in range(num_constraints)]
    
#             #All variables in the problem
#             names = event_names+ub_names+lb_names
#             #All variables are lower-bounded by 0
#             lb = [0 for i in range(len(names))]
#             #All variables are upper-bounded by infinity
#             ub = [cplex.infinity for i in range(len(names))]
    
#             #The objective function tries to minimize the total amount of relaxation
#             #in the problem
#             obj = [1.0]*len(names)
#             for i in range(len(event_names)):
#                 obj[i] = 0.0
    
#             #One weight per constraint
#             if len(stn_weights) == num_constraints:
#                 #print "Performing weighted constraint relaxation"
#                 weights = stn_weights
    
#                 #Assigns different weights to relaxing the bounds of different
#                 #temporal constraints
#                 for i in range(num_constraints):
#                     obj[num_events+i]=weights[i] #Upper bound
#                     obj[num_events+num_constraints+i]=weights[i]#Lower bound
#             #else:
#                # print "Performing unweighted constraint relaxation"
    
#             #Adds all variables and the objective function to perform
#             #continuous constrain relaxation
#             prob.variables.add(obj=obj,lb=lb, ub=ub, names=names)
    
#             #Represents the simple temporal constraints as a set of linear
#             #constraints
#             rows = [[] for i in range(2*num_constraints)]
#             rhs = [[] for i in range(2*num_constraints)]
    
#             row_names = ["c"+str(i) for i in range(2*num_constraints)]
#             senses = "L"*len(row_names)
    
#             # lb-dL<=end-start<=ub+dU
#             #start-end <= dL-lb
#             for index,stc in enumerate(stp.get_stn()):
#                 #Upper bound constraint
#                 rows[2*index] = [[stc.end_,stc.start_,"dU"+str(index)],[1.0,-1.0,-1.0]]
#                 rhs[2*index] = stc.ub_
#                 #Lower bound constraint
#                 rows[2*index+1] = [[stc.start_,stc.end_,"dL"+str(index)],[1.0,-1.0,-1.0]]
#                 rhs[2*index+1] = -stc.lb_
    
#             #Adds the STCs to the problem
#             prob.linear_constraints.add(lin_expr=rows, senses=senses,rhs=rhs, names=row_names)
    
#             #Solves the constrain relaxation problem
#             prob.solve()
    
#             if prob.solution.get_status() == prob.solution.status.infeasible:        
#                 return False,None,[],[]                                
#             else:
#                 #Get solution values
#                 values = prob.solution.get_values()
    
#                 #Upper bound relaxations
#                 dUs = values[num_events:(num_events+num_constraints)]
#                 #Lower bound relaxations
#                 dLs = values[(num_events+num_constraints):]
                
#                 return True,prob,dUs,dLs
                
    
#         except CplexError as exc:
#             print(exc)
#             return False,None,[],[]      
    
#     def get_prev(self):
#         return self._prev_problem
    
#     def reset(self):
#         self._prev_problem = None