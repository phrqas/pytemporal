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

Chance-constrained, strong controllability checker implemented using Gurobi

@author: Pedro Santana (psantana@mit.edu).
"""
from gurobipy import *
import numpy as np
import scipy.stats as st
from .gaussian_piecewise import get_gaussian_partition

class PARIS(object):
    """
    Class encapsulating the different functionality of PARIS, the Polynomial-time,
    RIsk-sensitive Scheduler.
    """
    def __init__(self,gaussian_div=5,gaussian_optimize_partition=False,
                 gaussian_lr=0.05,gaussian_tol=1e-5,gaussian_max_iter=10000,
                 gaussian_lb_sigma=None,gaussian_ub_sigma=None,
                 random_part_init=False,verbose=False):
        self.gaussian_div = gaussian_div
        self.gaussian_optimize_partition = gaussian_optimize_partition
        self.gaussian_lr = gaussian_lr
        self.gaussian_tol = gaussian_tol
        self.gaussian_max_iter = gaussian_max_iter
        self.gaussian_lb_sigma = gaussian_lb_sigma
        self.gaussian_ub_sigma = gaussian_ub_sigma
        self.random_part_init = random_part_init
        self.verbose = verbose

    def init_model(self,):
        """
        Initializes an empty optimization model
        """
        self.model = Model('paris_model')
        self.model.setParam('OutputFlag',1 if self.verbose else 0)
        self.modelSense = GRB.MINIMIZE #Minimize risk of STNU reformulation

    def schedule(self,tcs,makespan=False,cc=-1.0,lin_sched_obj=None,reset=True):
        """
        Automates the process of calling the different forms of the PARIS module.
        """
        if reset:
            self.init_model()

        if self.set_minimum_risk_strong_controllability(tcs):
            #Objective function is makespan
            if makespan:
                self.set_makespan_optimization()
            else:
                #Objective function has been specified by the user
                if lin_sched_obj != None:
                    self.set_linear_schedule_objective(lin_sched_obj)
            #Add a chance constraint
            if cc>=0.0:
                self.set_chance_constraint(cc)
            return self.solve()
        else:
            None,None,None

    def stnu_reformulation(self,pstnu_or_prog=None,makespan=False,cc=-1.0,digits=3):
        """
        Takes a PSTNU or RMPyL program and performs the appropriate STNU reformulation.
        It modifies the original object!
        """
        #The input is a list of temporal constraints (PSTNU)
        if isinstance(pstnu_or_prog,list):
            tcs = pstnu_or_prog
        #The input is an RMPyL program
        else:
            if len(pstnu_or_prog.choices)>0:
                print('\nWARNING: PARIS should not be used for conditional scheduling!')
            tcs = pstnu_or_prog.temporal_constraints

        squeeze_dict,risk_bound,sc_schedule = self.schedule(tcs,makespan,cc)
        if squeeze_dict != None:
            for tc,tc_dict in squeeze_dict.items():
                tc.set_stcu(round(tc_dict['lb'],digits),round(tc_dict['ub'],digits))
            return risk_bound,sc_schedule
        else:
            print('The %s is not strongly controllable'%('PSTNU' if isinstance(pstnu_or_prog,list) else 'RMPyL program'))
            return None,None

    def set_minimum_risk_strong_controllability(self,tcs):
        """
        Sets up the strong controllability problem for risk minimization.
        """
        #Extracts the temporal reference structure from the pSTPN, while adding
        #the required squeezing variables for probabilistic durations.
        self.sc_reform={'_baseline_risk_':0.0}
        self.tc_squeeze_var_dict={}
        requirements=[]
        for tc in tcs:
            if tc.type in ['uncontrollable_probabilistic','uncontrollable_bounded']:
                if tc.end.name in self.sc_reform:
                    #NOTE:some Zipcar tests had uncontrollable durations sharing
                    #end points, so I had to convert the exception into a failure in
                    #order to be able to run all the tests.
                    print('An event cannot be the end point of two uncontrollable durations.')
                    return False
                else:
                    self.sc_reform[tc.end.name]={'ref':tc.start.name,'lb':tc.lb,'ub':tc.ub}

                if tc.type=='uncontrollable_probabilistic':
                    dist_type=tc.distribution['type']
                    if dist_type=='uniform':
                        lb,ub=tc.distribution['lb'],tc.distribution['ub']
                        uniform_sc_reformulation(tc,lb,ub,self.model,self.sc_reform,
                                                 self.tc_squeeze_var_dict)
                    elif dist_type=='gaussian':
                        #FIXME:some Zipcar tests had distributions with variance equal to 0.0,
                        #which cause errors in the piecewise approximation. This is clearly
                        #inconsistent.
                        if tc.distribution['variance']>0.0:
                            smart_piecewise_gaussian_sc_reformulation(tc,self.model,self.sc_reform,
                                                                      self.tc_squeeze_var_dict,
                                                                      num_div=self.gaussian_div,
                                                                      optimize_partition=self.gaussian_optimize_partition,
                                                                      lr=self.gaussian_lr,
                                                                      tol=self.gaussian_tol,
                                                                      max_iter=self.gaussian_max_iter,
                                                                      gaussian_lb_sigma=self.gaussian_lb_sigma,
                                                                      gaussian_ub_sigma=self.gaussian_ub_sigma,
                                                                      random_part_init=self.random_part_init)
                    else:
                        raise TypeError('Gurobi only supports uniform and Gaussian distributions at the moment.')

            else:#Controllable duration
                requirements.append(tc)

        #Adds baseline risk as a constant to the objective (allows correct chance constraints)
        self.model.setObjective(self.model.getObjective()+self.sc_reform['_baseline_risk_'],GRB.MINIMIZE)
        self.model.update() # Integrate new variables

        #For strong controllability, only controllable temporal events are variables
        #in the optimization model. All uncontrollable time points should be replaced
        #by their maximum and minimum values.
        self.var_map={}; path_cache={}
        for tc in requirements:
            #Minimal and maximal values of temporal distances
            dv1 = ctg_path(tc.start.name,self.sc_reform,path_cache,self.var_map,
                           self.model,set())
            dv2 = ctg_path(tc.end.name,self.sc_reform,path_cache,self.var_map,
                           self.model,set())
            df_min,df_max = diff_min_max(dv2,dv1,self.sc_reform,self.var_map)

            #Strongly controllable reformulation
            self.model.addConstr(df_min>=tc.lb)
            self.model.addConstr(df_max<=tc.ub)

        self.risk_bound = self.model.getObjective() #Gets the risk bound expression
        self.model.update()

        return True

    def set_makespan_optimization(self,tight_risk=True,mult=1000.0):
        """
        Adds makespan to the objective and constraints.
        """
        #Surrogate objective that ensures a tight risk bound with approximate
        #makespan
        if tight_risk:
            M = len(self.sc_reform)*mult
            t = self.model.addVar(vtype=GRB.CONTINUOUS,lb=0.0,obj=M) #Makespan
            self.model.update()
        else: #Optimizes makespan directly
            t = self.model.addVar(vtype=GRB.CONTINUOUS,lb=0.0) #Makespan
            self.model.update()
            self.model.setObjective(t,GRB.MINIMIZE)

        for ec in self.var_map.values():
            self.model.addConstr(t-ec>=0.0) #All controllable events should be less

        self.model.update()
        return t

    def set_linear_schedule_objective(self,lin_sched_obj):
        """
        Sets a generic linear objective over the controllable schedule variables.
        """
        valid_objective=True
        if isinstance(lin_sched_obj,dict):
            for field in ['events','coefficients','maximize']:
                if not field in lin_sched_obj:
                    valid_objective=False
                    print('\nERROR: missing field '+field+'\n')
                    break
        else:
            valid_objective=False

        if valid_objective:
            #Extracts the names of the events, whether they are given as event
            #objects or their names directly.
            ec_names = [e if isinstance(e,str) else e.name for e in lin_sched_obj['events']]

            #Constructs the objective
            obj = quicksum([lin_sched_obj['coefficients'][i]*self.var_map[n] for i,n in enumerate(ec_names)])

            #Updates the model
            self.model.setObjective(obj,GRB.MAXIMIZE if lin_sched_obj['maximize'] else GRB.MINIMIZE)
            self.model.update()
        else:
            print('\nERROR: Invalid specification of linear schedule objective as dictionary.')
            print('\t"events": list of event objects or names involved in the objective')
            print('\t"coefficients": list of corresponding coefficients for the events in the objective.')
            print('\t"maximize": whether to maximize (TRUE) or minimize (FALSE).')
            obj = None

        return obj

    def set_chance_constraint(self,theta):
        """
        Adds a chance constraint to the model.
        """
        cc = self.model.addConstr(self.risk_bound<=theta)
        return cc

    def solve(self):
        """
        Solves the internal optimization problem and returns the solution.
        """
        self.model.optimize()#Solves LP with SC constraints

        if self.model.status == GRB.status.OPTIMAL: #Found optimal squeezing
            #Returns amount by which temporal constraints should be squeezed,
            #but only for the ones with non-zero (or very close to zero)
            #squeezing
            squeeze_dict={}
            for tc,tc_dict in self.tc_squeeze_var_dict.items():
                new_lb_ub_bounds=[]
                for bound in [tc_dict['lb'],tc_dict['ub']]:
                    if isinstance(bound,gurobipy.LinExpr):
                        new_lb_ub_bounds.append(bound.getValue())
                    elif isinstance(bound,gurobipy.Var):
                        new_lb_ub_bounds.append(bound.X)
                    else:
                        new_lb_ub_bounds.append(bound)
                risk= tc_dict['risk_function'](tc,new_lb_ub_bounds[0],new_lb_ub_bounds[1])
                squeeze_dict[tc]={'lb':new_lb_ub_bounds[0],'ub':new_lb_ub_bounds[1],'risk':risk}

            #One possible strongly controllable schedule
            sc_schedule={event_id:var.X for event_id,var in self.var_map.items()}

            # Returns the objective with the constant _baseline_risk_ added to it already
            return squeeze_dict,self.risk_bound.getValue(),sc_schedule
            # return squeeze_dict,sc_model.ObjVal+sc_reform['_baseline_risk_'],sc_schedule

        else: #No relaxation found
            return None,None,None



def ctg_path(event_name,sc_reform,path_cache,var_map,model,prev_events):
    """
    Recursively computes the controllable and contigent events that influence
    the schedule of a given event.
    """
    if event_name in path_cache:#If solution has been already computed, use it
        return path_cache[event_name]
    else:
        if event_name in sc_reform: #End point of uncontrollable duration
            if event_name in prev_events:
                raise RuntimeError('Contigent duration loop detected!')
            else:
                prev_events.add(event_name)
            path_ref = ctg_path(sc_reform[event_name]['ref'],sc_reform,path_cache,var_map,model,prev_events)
            path = [event_name]+path_ref
        else: #Controllable event
            if not event_name in var_map:#1-to-1 mapping between events and variables
                var_map[event_name]=model.addVar(vtype=GRB.CONTINUOUS,lb=0.0)
                model.update()
            path = [event_name]

        path_cache[event_name]=path #Caches solution for future use
        return path

def diff_min_max(dv2,dv1,sc_reform,var_map):
    """
    Computes the maximum and the minimum differences between two sequences of events.
    """
    dvs = [dv1[:-1],dv2[:-1]] #All contingent durations
    mins = [0.0,0.0]
    maxs = [0.0,0.0]
    for i in range(2):
        for ev in [e for e in dvs[i] if not e in dvs[1-i]]:
            mins[i] += sc_reform[ev]['lb']
            maxs[i] += sc_reform[ev]['ub']
    ec1 = var_map[dv1[-1]]; ec2 = var_map[dv2[-1]]
    diff_min = ec2-ec1+mins[1]-maxs[0]
    diff_max = ec2-ec1+maxs[1]-mins[0]
    return diff_min, diff_max

def uniform_sc_reformulation(tc,lb,ub,model,sc_reform,tc_squeeze_var_dict):
    """
    Creates the necessary variables and constraints to treat strong controllability
    of probabilistic durations with uniform distribution.
    """
    pdf = 1.0/(ub-lb) #PDF associated with squeezing this variable
    lb_var = model.addVar(vtype=GRB.CONTINUOUS,lb=0.0,ub=ub-lb,obj=pdf)
    ub_var = model.addVar(vtype=GRB.CONTINUOUS,lb=0.0,ub=ub-lb,obj=pdf)
    model.update()
    #The relaxation cannot make the upper bound bigger than the
    #lower bound.
    model.addConstr(lb_var+ub_var<=ub-lb)
    #Updates the strong controllability reformulation dictionary
    sc_reform[tc.end.name]['lb']+=lb_var
    sc_reform[tc.end.name]['ub']+=-ub_var
    #Updates the correspondence between uncontrollable temporal constraints
    #and squeezing variables
    tc_squeeze_var_dict[tc]={'lb':sc_reform[tc.end.name]['lb'],
                             'ub':sc_reform[tc.end.name]['ub'],
                             'risk_function':uniform_squeeze_risk}

def smart_piecewise_gaussian_sc_reformulation(tc,model,sc_reform,tc_squeeze_var_dict,
                                               num_div=5,optimize_partition=False,
                                               lr=0.05,tol=1e-5,max_iter=10000,
                                               gaussian_lb_sigma=None,
                                               gaussian_ub_sigma=None,
                                               random_part_init=False):
    """
    Generates an upper bound for a Gaussian CDF by performing a piecewise constraint
    approximation of its PDF (therefore approximating the CDF by straight line
    segments).
    """
    mean,var = tc.distribution['mean'],tc.distribution['variance']
    stdev = np.sqrt(var)
    lb_breaks, ub_breaks = get_gaussian_partition(mean=mean,var=var,num_div=num_div,
                                                  lb_sigma=gaussian_lb_sigma,
                                                  ub_sigma=gaussian_ub_sigma,
                                                  optimize_partition=optimize_partition,
                                                  random_part_init=random_part_init,
                                                  lr=lr,tol=tol,max_iter=max_iter)

    lb_vars=[]
    for i in range(1,len(lb_breaks)):
        lb_vars.append(model.addVar(vtype=GRB.CONTINUOUS,lb=0.0,ub=lb_breaks[i]-lb_breaks[i-1],
                                    obj=st.norm.pdf(lb_breaks[i],loc=mean,scale=stdev)))
    ub_vars=[]
    for i in range(1,len(ub_breaks)):
        ub_vars.append(model.addVar(vtype=GRB.CONTINUOUS,lb=0.0,ub=ub_breaks[i]-ub_breaks[i-1],
                                    obj=st.norm.pdf(ub_breaks[i-1],loc=mean,scale=stdev)))

    #Adds the variables to the model
    model.update()

    lower_bound = lb_breaks[0]+quicksum(lb_vars) #Total lower bound
    upper_bound = ub_breaks[-1]-quicksum(ub_vars)#Total upper bound

    #model.addConstr(lower_bound-upper_bound<=0.0)#Lower bound cannot be bigger than upper bound

    #Updates the strong controllability reformulation dictionary
    sc_reform[tc.end.name]['lb']=lower_bound
    sc_reform[tc.end.name]['ub']=upper_bound

    #Baseline risk of cutting a Gaussian
    sc_reform['_baseline_risk_']+=st.norm.cdf(lb_breaks[0],loc=mean,scale=stdev)+1.0-st.norm.cdf(ub_breaks[-1],loc=mean,scale=stdev)

    #Updates the correspondence between uncontrollable temporal constraints
    #and squeezing variables
    tc_squeeze_var_dict[tc]={'lb':sc_reform[tc.end.name]['lb'],
                             'ub':sc_reform[tc.end.name]['ub'],
                             'risk_function':gaussian_squeeze_risk}


def uniform_squeeze_risk(tc,new_lb,new_ub):
    """
    Risk for squeezing uniform distributions.
    """
    ub_squeeze=tc.ub-new_ub; lb_squeeze=new_lb-tc.lb
    if ub_squeeze>=0.0 and lb_squeeze>=0.0:
        return (1.0/(tc.ub-tc.lb))*(ub_squeeze+lb_squeeze)
    else:
        raise ValueError('Invalid squeezing of uniform distribution:[%f,%f]'%(lb_squeeze,ub_squeeze))

def gaussian_squeeze_risk(tc,new_lb,new_ub):
    """
    Risk for squeezing Gaussian distributions.
    """
    mean = tc.distribution['mean']; stdev=np.sqrt(tc.distribution['variance'])
    return st.norm.cdf(new_lb,loc=mean,scale=stdev)+1.0-st.norm.cdf(new_ub,loc=mean,scale=stdev)
