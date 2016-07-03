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
from gaussian_piecewise import optimize_piecewise_gaussian_approximation

def strongly_controllable_stp(tcs,gaussian_approx='smart_piecewise',gaussian_div=5,
                              gaussian_optimize_partition=False,gaussian_lr=0.05,
                              gaussian_tol=1e-5,gaussian_max_iter=10000):
    """
    Returns the minimum risk pSTN with a strongly controllable STNU reformulation.
    """
    sc_model = Model('sc_model')
    sc_model.setParam('OutputFlag',0) #No output to console
    sc_model.modelSense = GRB.MINIMIZE #Minimize risk of STNU reformulation

    if gaussian_approx=='smart_piecewise':
        gaussian_sc_reformulation=_smart_piecewise_gaussian_sc_reformulation
    elif gaussian_approx=='uniform':
        print('WARNING: This approximation of Gaussian distributions should be considered deprecated.')
        gaussian_sc_reformulation=_gaussian_as_uniform_sc_reformulation
    elif gaussian_approx=='piecewise':
        print('WARNING: This approximation of Gaussian distributions should be considered deprecated.')
        gaussian_sc_reformulation=_piecewise_gaussian_sc_reformulation
    elif gaussian_approx=='beyond_mean':
        print('WARNING: This approximation of Gaussian distributions is not yet working.')
        gaussian_sc_reformulation=_smart_piecewise_gaussian_beyond_mean_sc_reformulation
    else:
        raise ValueError(gaussian_approx+' is not a valid type of Gaussian approximation')

    #Extracts the temporal reference structure from the pSTPN, while adding
    #the required squeezing variables for probabilistic durations.
    sc_reform={'_baseline_risk_':0.0}; tc_squeeze_var_dict={}; requirements=[]
    for tc in tcs:
        if tc.type in ['uncontrollable_probabilistic','uncontrollable_bounded']:
            if tc.end.name in sc_reform:
                #FIXME:some Zipcar tests had uncontrollable durations sharing
                #end points, so I had to convert the exception into a failure in
                #order to be able to run all the tests.
                print('An event cannot be the end point of two uncontrollable durations.')
                return None,None,None
                #raise TypeError('An event cannot be the end point of two uncontrollable durations.')
            else:
                sc_reform[tc.end.name]={'ref':tc.start.name,'lb':tc.lb,'ub':tc.ub}

            if tc.type=='uncontrollable_probabilistic':
                dist_type=tc.distribution['type']
                if dist_type=='uniform':
                    lb,ub=tc.distribution['lb'],tc.distribution['ub']
                    _uniform_sc_reformulation(tc,lb,ub,sc_model,sc_reform,tc_squeeze_var_dict)
                elif dist_type=='gaussian':
                    #FIXME:some Zipcar tests had distributions with variance equal to 0.0,
                    #which cause errors in the piecewise approximation. This is clearly
                    #inconsistent.
                    if tc.distribution['variance']>0.0:
                        gaussian_sc_reformulation(tc,sc_model,sc_reform,
                                                  tc_squeeze_var_dict,
                                                  num_div=gaussian_div,
                                                  optimize_partition=gaussian_optimize_partition)
                else:
                    raise TypeError('Gurobi only supports uniform and Gaussian distributions at the moment.')

        else:#Controllable duration
            requirements.append(tc)

            # TODO: According to my conversation with Simon today (08/25/2015),
            # this 'cleaverness' is actually wrong: if the next start event is,
            # in fact, the start of some activity that must be dispatched by
            # Pike, we can't just simply say 'start d units after the previous',
            # since this would be dynamic, and not strong controllability.
            #
            # #A [d,d] controllable temporal duration effectively represents that
            # #its endpoint is a constant shift d away from the start point, and a
            # #new variable should not be created. By adding the endpoint to the sc_reform
            # #dictionary, we can ensure that the endpoint is correctly characterized
            # #as an uncontrollable time point, should the start be uncontrollable
            # #as well.
            # if (np.isclose(tc.lb,0.0) and np.isclose(tc.ub,0.0)) or (improved_sc and np.isclose(tc.ub-tc.lb,0.0)):
            #     sc_reform[tc.end.name]={'ref':tc.start.name,'lb':tc.lb,'ub':tc.lb}
            # else:
            #     requirements.append(tc)

    sc_model.update() # Integrate new variables

    #For strong controllability, only controllable temporal events are variables
    #in the optimization model. All uncontrollable time points should be replaced
    #by their maximum and minimum values.
    var_map={}; reform_cache={}
    for tc in requirements:
        #Minimal and maximal values of the temporal event variables
        min_start,max_start = _recursive_min_max(tc.start.name,sc_reform,
                                                 reform_cache,var_map,sc_model)
        min_end,max_end = _recursive_min_max(tc.end.name,sc_reform,reform_cache,
                                             var_map,sc_model)
        #Strongly controllable reformulation
        lb_const = sc_model.addConstr(min_end-max_start>=tc.lb)
        ub_const = sc_model.addConstr(max_end-min_start<=tc.ub)

    sc_model.optimize()#Performs minimum risk SC reformulation

    if sc_model.status == GRB.status.OPTIMAL: #Found optimal squeezing
        #Returns amount by which temporal constraints should be squeezed,
        #but only for the ones with non-zero (or very close to zero)
        #squeezing
        squeeze_dict={}
        for tc,tc_dict in tc_squeeze_var_dict.items():
            num_bounds=[]
            for bound in [tc_dict['lb'],tc_dict['ub']]:
                if isinstance(bound,gurobipy.LinExpr):
                    num_bounds.append(bound.getValue())
                elif isinstance(bound,gurobipy.Var):
                    num_bounds.append(bound.X)
                else:
                    num_bounds.append(bound)
            risk= tc_dict['risk_function'](tc,num_bounds[0],num_bounds[1])
            squeeze_dict[tc]={'lb':num_bounds[0],'ub':num_bounds[1],'risk':risk}

        #One possible strongly controllable schedule
        sc_schedule={event_id:var.X for event_id,var in var_map.items()}

        return squeeze_dict,sc_model.ObjVal+sc_reform['_baseline_risk_'],sc_schedule
    else: #No relaxation found
        return None,None,None

def _recursive_min_max(event_name,sc_reform,reform_cache,var_map,model):
    """
    Recursively determines the minimum and maximum values of uncontrollable
    time points with respect to controllable references.
    """
    if event_name in reform_cache:#If solution has been already computed, use it
        return reform_cache[event_name]
    else:
        if event_name in sc_reform: #End point of uncontrollable duration
            min_ref,max_ref = _recursive_min_max(sc_reform[event_name]['ref'],
                                                 sc_reform,reform_cache,var_map,
                                                 model)
            lower_bound = min_ref + sc_reform[event_name]['lb']
            upper_bound = max_ref + sc_reform[event_name]['ub']
        else: #Controllable event
            if not event_name in var_map:#1-to-1 mapping between events and variables
                var_map[event_name]=model.addVar(vtype=GRB.CONTINUOUS,lb=0.0)
                model.update()

            lower_bound = upper_bound = var_map[event_name]

        reform_cache[event_name]=[lower_bound,upper_bound]
        return reform_cache[event_name]

def _uniform_squeeze_risk(tc,new_lb,new_ub):
    """
    Risk for squeezing uniform distributions.
    """
    ub_squeeze=tc.ub-new_ub; lb_squeeze=new_lb-tc.lb
    if ub_squeeze>=0.0 and lb_squeeze>=0.0:
        return (1.0/(tc.ub-tc.lb))*(ub_squeeze+lb_squeeze)
    else:
        raise ValueError('Invalid squeezing of uniform distribution:[%f,%f]'%(lb_squeeze,ub_squeeze))

def _uniform_sc_reformulation(tc,lb,ub,model,sc_reform,tc_squeeze_var_dict):
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
                             'risk_function':_uniform_squeeze_risk}

def _gaussian_squeeze_risk(tc,new_lb,new_ub):
    """
    Risk for squeezing Gaussian distributions.
    """
    mean = tc.distribution['mean']; stdev=np.sqrt(tc.distribution['variance'])
    return st.norm.cdf(new_lb,loc=mean,scale=stdev)+1.0-st.norm.cdf(new_ub,loc=mean,scale=stdev)

def _smart_piecewise_gaussian_sc_reformulation(tc,model,sc_reform,tc_squeeze_var_dict,
                                               num_div=5,optimize_partition=False,
                                               lr=0.05,tol=1e-5,max_iter=10000):
    """
    Generates an upper bound for a Gaussian CDF by performing a piecewise constraint
    approximation of its PDF (therefore approximating the CDF by straight line
    segments). It generates a MILP model.
    """
    mean,var = tc.distribution['mean'],tc.distribution['variance']
    stdev = np.sqrt(var)

    if optimize_partition:
        part = [mean-i*stdev for i in range(-num_div,num_div+1) if mean-i*stdev > 0.0]
        opt_part = list(optimize_piecewise_gaussian_approximation(part,mean,var,lr,tol,max_iter,False))
        mean_idx = opt_part.index(mean)
        lb_breaks = opt_part[i:mean_idx+1]; ub_breaks = opt_part[mean_idx:]
    else:
        #Breakpoints for piecewise approximation of the lower bound penalty (overestimate
        #of the area under the first half of the Gaussian).
        lb_breaks=[mean-i*stdev for i in range(num_div,-1,-1) if (mean-i*stdev)>0.0]

        #Breakpoints for piecewise approximation of the upper bound penalty (should
        #overstimate the area under the second half of the Gaussian).
        ub_breaks=[mean+i*stdev for i in range(num_div+1)]

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
                             'risk_function':_gaussian_squeeze_risk}


#################################################################################################
#TODO: this function should be considered DEPRECATED
def _gaussian_as_uniform_sc_reformulation(tc,model,sc_reform,tc_squeeze_var_dict,stdev_mult=3.0):
    """
    Approximates a Gaussian by a single uniform distribution with 3sigma bounds.
    """
    mean,stdev = tc.distribution['mean'],np.sqrt(tc.distribution['variance'])
    lb = max(0.0,mean-stdev*stdev_mult)
    ub = mean+stdev*stdev_mult
    tc.lb=sc_reform[tc.end.name]['lb']=lb
    tc.ub=sc_reform[tc.end.name]['ub']=ub
    _uniform_sc_reformulation(tc,lb,ub,model,sc_reform,tc_squeeze_var_dict)

    #Updates the risk computation function, which should be Gaussian
    tc_squeeze_var_dict[tc]['risk_function']=_gaussian_squeeze_risk

    #Ensures that the uniform approximation always overestimates the risk, by
    #limiting the relaxation variables to go up to the point where the uniform
    #PDF intercepts the true Gaussian PDF.
    uniform_pdf = 1.0/(ub-lb)
    if st.norm.pdf(mean,loc=mean,scale=stdev)>uniform_pdf:
        sigma_sqrt_2 = stdev*np.sqrt(2.0)
        dist_mean = sigma_sqrt_2*np.sqrt(np.log(1.0/(uniform_pdf*sigma_sqrt_2*np.sqrt(np.pi))))
        model.addConstr(sc_reform[tc.end.name]['lb']<=mean-dist_mean)
        model.addConstr(sc_reform[tc.end.name]['ub']>=mean+dist_mean)



#FIXME: This formulation still needs fixing, and doesn't quite work.
def _smart_piecewise_gaussian_beyond_mean_sc_reformulation(tc,model,sc_reform,tc_squeeze_var_dict,num_div=5):
    """
    Generates an upper bound for a Gaussian CDF by performing a piecewise constraint
    approximation of its PDF (therefore approximating the CDF by straight line
    segments). It generates a MILP model.
    """
    mean,var = tc.distribution['mean'],tc.distribution['variance']
    stdev = np.sqrt(var)

    #Breakpoints for piecewise approximation of the lower bound penalty (overestimate
    #of the area under the first half of the Gaussian).
    lb_breaks=[mean-i*stdev for i in range(num_div,-1,-1) if (mean-i*stdev)>0.0]

    #Breakpoints for piecewise approximation of the upper bound penalty (should
    #underestimate the area under the second half of the Gaussian).
    ub_breaks=[mean+i*stdev for i in range(num_div+1)]

    lb_vars=[]
    for i in range(1,len(lb_breaks)):
        lb_vars.append(model.addVar(vtype=GRB.CONTINUOUS,lb=0.0,ub=lb_breaks[i]-lb_breaks[i-1],
                                    obj=st.norm.pdf(lb_breaks[i],loc=mean,scale=stdev)))
    #Adds lower bound squeezing variables beyond the mean, which approximate the
    #PDF horribly in order to avoid integer variables
    for i in range(1,len(ub_breaks)):
        lb_vars.append(model.addVar(vtype=GRB.CONTINUOUS,lb=0.0,ub=ub_breaks[i]-ub_breaks[i-1],
                                    obj=st.norm.pdf(mean,loc=mean,scale=stdev)*(1.001**i)))

    ub_vars=[]
    for i in range(1,len(ub_breaks)):
        ub_vars.append(model.addVar(vtype=GRB.CONTINUOUS,lb=0.0,ub=ub_breaks[i]-ub_breaks[i-1],
                                    obj=st.norm.pdf(ub_breaks[i],loc=mean,scale=stdev)))
    #Adds upper bound squeezing variables beyond the mean, which approximate the
    #PDF horribly in order to avoid integer variables
    lb_breaks.reverse()
    for i in range(1,len(lb_breaks)):
        ub_vars.append(model.addVar(vtype=GRB.CONTINUOUS,lb=0.0,ub=lb_breaks[i-1]-lb_breaks[i],
                                    obj=st.norm.pdf(mean,loc=mean,scale=stdev)*(1.001**i)))

    #Adds the variables to the model
    model.update()

    lower_bound = lb_breaks[0]+quicksum(lb_vars) #Total lower bound
    upper_bound = ub_breaks[-1]-quicksum(ub_vars)#Total upper bound

    model.addConstr(lower_bound-upper_bound<=0.0)#Lower bound cannot be bigger than upper bound

    #Updates the strong controllability reformulation dictionary
    sc_reform[tc.end.name]['lb']=lower_bound
    sc_reform[tc.end.name]['ub']=upper_bound

    #Updates the correspondence between uncontrollable temporal constraints
    #and squeezing variables
    tc_squeeze_var_dict[tc]={'lb':sc_reform[tc.end.name]['lb'],
                             'ub':sc_reform[tc.end.name]['ub'],
                             'risk_function':_gaussian_squeeze_risk}


# DEPRECATED: This should probably not be used anymore
#
# def _piecewise_gaussian_sc_reformulation(tc,model,sc_reform,tc_squeeze_var_dict,num_div=2):
#     """
#     Generates an upper bound for a Gaussian CDF by performing a piecewise constraint
#     approximation of its PDF (therefore approximating the CDF by straight line
#     segments). It generates a MILP model.
#     """
#     mean,var = tc.distribution['mean'],tc.distribution['variance']
#     stdev = np.sqrt(var)

#     #Lower bound variable is upper bounded by the mean and lower bounded by 0
#     lb_var = model.addVar(vtype=GRB.CONTINUOUS,lb=0.0,ub=mean)
#     #Upper bound variable is lower bounded by the mean
#     ub_var = model.addVar(vtype=GRB.CONTINUOUS,lb=mean)
#     #Adds the variables to the model
#     model.update()

#     #Breakpoints for piecewise approximation of the lower bound penalty (overestimate
#     #of the area under the first half of the Gaussian). A point is added at 0.0 to
#     #make sure all values below the last division incur the same overestimated penalty
#     lb_breaks=[mean-i*stdev for i in range(num_div,0,-1) if (mean-i*stdev)>0.0]
#     lb_breaks=[lb_breaks[0]*0.99]+lb_breaks
#     #First two points have the same risk, which is given by the true Gaussian CDF
#     #up to the last division.
#     lb_risks=[0.0]*len(lb_breaks)
#     lb_risks[0]=lb_risks[1]=st.norm.cdf(lb_breaks[1],loc=mean,scale=stdev)
#     #Overestimates the area at each point by taking the value of the PDF at the
#     #right-most break
#     for i in range(2,len(lb_risks)):
#         lb_risks[i]=st.norm.cdf(lb_breaks[i-1],loc=mean,scale=stdev)+st.norm.pdf(lb_breaks[i],loc=mean,scale=stdev)*(lb_breaks[i]-lb_breaks[i-1])

#     #Breakpoints for piecewise approximation of the upper bound penalty (should
#     #understimate the area under the second half of the Gaussian). A point is added
#     #after the last break to make sure all values beyond the last division incur
#     #the same overestimated penalty
#     ub_breaks=[mean+i*stdev for i in range(num_div+1)]
#     ub_breaks.append(ub_breaks[-1]*1.01)

#     ub_risks=[0.0]*len(ub_breaks)
#     ub_risks[0]=0.5
#     for i in range(1,len(ub_risks)-1):
#         ub_risks[i]=(1.0-st.norm.cdf(ub_breaks[i-1],loc=mean,scale=stdev))-st.norm.pdf(ub_breaks[i],loc=mean,scale=stdev)*(ub_breaks[i]-ub_breaks[i-1])
#     #Last two divisions have the same value to ensure a flat overestimate
#     ub_risks[-1]=ub_risks[-2]

#     #Sets the piecewise linear objective for the lower bound
#     model.setPWLObj(lb_var,lb_breaks,lb_risks)
#     #Sets the piecewise linear objective for the upper bound
#     model.setPWLObj(ub_var,ub_breaks,ub_risks)

#     #Updates the strong controllability reformulation dictionary
#     sc_reform[tc.end.name]['lb']=lb_var
#     sc_reform[tc.end.name]['ub']=ub_var

#     #Updates the correspondence between uncontrollable temporal constraints
#     #and squeezing variables
#     tc_squeeze_var_dict[tc]={'lb':sc_reform[tc.end.name]['lb'],
#                              'ub':sc_reform[tc.end.name]['ub'],
#                              'risk_function':_gaussian_squeeze_risk}


#     # #DEBUG stuff
#     # #
#     # coords_low = list(np.linspace(start=lb_breaks[0],stop=lb_breaks[-1]))
#     # cdf_values_low = [st.norm.cdf(x,loc=mean,scale=stdev) for x in coords_low]

#     # coords_up = list(np.linspace(start=ub_breaks[0],stop=ub_breaks[-1]))
#     # cdf_values_up = [1.0-st.norm.cdf(x,loc=mean,scale=stdev) for x in coords_up]

#     # # ipdb.set_trace()

#     # plt.figure()
#     # plt.plot(lb_breaks+ub_breaks,lb_risks+ub_risks,'b-o',
#     #          coords_low+coords_up,cdf_values_low+cdf_values_up,'r-o')
#     # plt.grid()

#     # # plt.figure()
#     # # plt.plot(lb_breaks,lb_risks,'b-o')
#     # # plt.grid()

#     # # plt.figure()
#     # # plt.plot(ub_breaks,ub_risks,'r-o')
#     # # plt.grid()

#     # plt.show()
