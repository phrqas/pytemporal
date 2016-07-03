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
pyTemporal: A Python package for temporal consistency and scheduling.

Suite of simple examples.

@author: Pedro Santana (psantana@mit.edu).
"""
from pytemporal.interface import PyTemporal,TemporalConstraint
from pytemporal.paris import PARIS
from pytemporal.utils import from_json_to_rmpyl_benchmark, load_pickle_file
from rmpyl.defs import Event
import sys,os,time,pickle

def check_consistency(tcs,example_name=''):
    """Uses different methods to very consistency of an STN."""
    pt_nx = PyTemporal(stn_consistency='networkx')
    pt_gur = PyTemporal(stn_consistency='gurobi')
    for tc in tcs:
        pt_nx.add_temporal_constraint(tc)
        pt_gur.add_temporal_constraint(tc)

    start = time.time()
    networkx_consistent = pt_nx.consistent_stn()
    nx_elapsed = time.time()-start

    start = time.time()
    gurobi_consistent = pt_gur.consistent_stn()
    gr_elapsed = time.time()-start

    print('\n***** '+example_name)
    if networkx_consistent:
        print('Networkx: consistent STN!')
    else:
        print('Networkx: inconsistent STN!')
    print('Elapsed time [ms]: %.3f\n'%(nx_elapsed*1000.0))

    if gurobi_consistent:
        print('Gurobi: consistent STN!')
    else:
        print('Gurobi: inconsistent STN!')
    print('Elapsed time [ms]: %.3f\n'%(gr_elapsed*1000.0))

    if networkx_consistent!=gurobi_consistent:
        print('WARNING: Networkx and Gurobi disagree!')
        return None,None
    else:
        return networkx_consistent,{'networkx':pt_nx,'gurobi':pt_gur}

def check_cc_strong_controllability(tcs,example_name='',write_tpns=False,
                                    gaussian_approx='smart_piecewise',gaussian_div=5,
                                    gaussian_optimize_partition=False,gaussian_lr=0.05,
                                    gaussian_tol=1e-4,gaussian_max_iter=1000,
                                    cc=-1.0,makespan=False):
    """
    Checks the chance-constrained strong controllability of a pSTN.
    """
    pt = PyTemporal()

    start_time = time.time()
    paris = PARIS(gaussian_div,gaussian_optimize_partition,gaussian_lr,
                  gaussian_tol,gaussian_max_iter)

    paris.init_model()
    if paris.set_minimum_risk_strong_controllability(tcs):
        if makespan:
            paris.set_makespan_optimization()
        if cc>=0.0:
            paris.set_chance_constraint(cc)
        squeeze_dict,risk_bound,sc_schedule = paris.solve()
    else:
        squeeze_dict=None

    if squeeze_dict!=None:
        risk_bound = min(risk_bound,1.0) #The bound must saturate at 1 (almost sure to fail)

    elapsed = time.time()-start_time

    print('\n***** '+example_name)
    if squeeze_dict!=None:
        print('Found SC reformulation in %.3f ms!'%(elapsed*1000.0))
        print('These are the squeezings:')
        prob_success=1.0
        for tc,tc_dict in squeeze_dict.items():
            print('\t%s [%.1f,%.1f]->[%.1f,%.1f]:\tRisk=%.4f%%'%(tc.name,tc.lb,tc.ub,tc_dict['lb'],tc_dict['ub'],tc_dict['risk']*100.0))
            prob_success*=(1.0-tc_dict['risk'])
        print('\nThis is the schedule:')
        for ev_name, ev_time in sc_schedule.items():
            print('\t%s: %.2f s'%(ev_name,ev_time))

        risk_indep = (1.0-prob_success)
        print('\nTotal scheduling risk (assuming independent stochastic durations): %.4f%%'%(risk_indep*100.0))
        print('Linear risk bound: %.4f%%'%(risk_bound*100.0))

        if risk_bound<risk_indep:
            raise ValueError('Boole\'s bound %f less than (independent) risk %f'%(risk_bound,risk_indep))
        else:
            risk_gap = risk_bound-risk_indep
            print('Bound gap: %f%%'%((risk_gap)*100.0))

        if write_tpns:
            sc_stnu=[]
            for tc in tcs:
                if tc.type=='controllable':
                    sc_stnu.append(tc)
                else:
                    if tc in squeeze_dict:
                        lb = tc_dict['lb']
                        ub = tc_dict['ub']
                    else:
                        lb = tc.lb
                        ub = tc.ub

                    sc_stnu.append(TemporalConstraint(start=tc.start,end=tc.end,
                                                      ctype='uncontrollable_bounded',
                                                      lb=lb, ub=ub))
            prog = pt.to_rmpyl(sc_stnu)
            prog.to_ptpn(filename='paris_sc_%s.tpn'%(example_name))
    else:
        prob_success=risk_bound=risk_gap=-1
        print('Strongly controllable reformulation failed in %.3f ms...'%(elapsed*1000.0))

    if write_tpns:
        prog = pt.to_rmpyl(tcs)
        prog.to_ptpn(filename='paris_pstn_%s.tpn'%(example_name))

    return prob_success,elapsed,risk_bound,risk_gap

def check_cc_strong_controllability_rmpyl(rmpyl_pickle,write_tpns=False):
    """
    Loads and checks if an RMPyL program is cc strongly controllable.
    """
    prog = load_pickle_file(rmpyl_pickle)
    pt=PyTemporal()
    tcs=pt.from_rmpyl(prog)
    check_cc_strong_controllability(tcs,example_name=prog.name,write_tpns=write_tpns)

def stn_consistency_consistent():
    """
    Simple consistent example.
    """
    events = [Event(name='e%d'%(i)) for i in range(3)]
    tcs = [TemporalConstraint(start=events[0],end=events[1],ctype='controllable',lb=2.0,ub=5.0),
           TemporalConstraint(start=events[1],end=events[2],ctype='controllable',lb=2.0,ub=5.0),
           TemporalConstraint(start=events[0],end=events[2],ctype='controllable',lb=0.0,ub=20.0)]
    consistent,interfaces = check_consistency(tcs,'simple_consistent')

    pt = interfaces['gurobi']
    prog = pt.to_rmpyl(tcs)
    prog.to_ptpn(filename='paris_simple_consistent.tpn')

def stn_consistency_inconsistent():
    """
    Simple inconsistent example
    """
    events = [Event(name='e%d'%(i)) for i in range(3)]
    tcs = [TemporalConstraint(start=events[0],end=events[1],ctype='controllable',lb=7.0,ub=10.0),
           TemporalConstraint(start=events[1],end=events[2],ctype='controllable',lb=4.0,ub=10.0),
           TemporalConstraint(start=events[0],end=events[2],ctype='controllable',lb=0.0,ub=10.0)]
    consistent,interfaces = check_consistency(tcs,'simple_inconsistent')

    pt = interfaces['gurobi']
    prog = pt.to_rmpyl(tcs)
    prog.to_ptpn(filename='paris_simple_inconsistent.tpn')

def stn_relaxation():
    """
    Example of performing minimum-cost relaxation to restore STN feasibility.
    """
    events = [Event(name='e%d'%(i)) for i in range(4)]
    tcs = [TemporalConstraint(start=events[0],end=events[1],ctype='controllable',lb=7.0,ub=10.0),
           TemporalConstraint(start=events[1],end=events[2],ctype='controllable',lb=4.0,ub=10.0),
           TemporalConstraint(start=events[0],end=events[2],ctype='controllable',lb=0.0,ub=10.0),
           TemporalConstraint(start=events[0],end=events[3],ctype='controllable',lb=2.0,ub=3.0),
           TemporalConstraint(start=events[3],end=events[2],ctype='controllable',lb=3.0,ub=3.0)]
    consistent,interfaces = check_consistency(tcs,'stn_relaxation')

    pt = interfaces['gurobi']
    prog = pt.to_rmpyl(tcs)
    prog.to_ptpn(filename='paris_stn_relaxation_prior.tpn')

    if consistent==False:
        print('* Inconsistent STN. Using Gurobi to perform relaxation.')
        relax_dict,relax_cost = pt.stn_relaxation(tcs)
        if relax_dict!=None:
            print('Relaxation worked! Minimum cost= %.3f'%(relax_cost))
            for tc,(relax_lb,relax_ub) in relax_dict.items():
                print('\t%s [%.1f,%.1f]->[%.1f,%.1f]'%(tc.name,tc.lb,tc.ub,tc.lb-relax_lb,tc.ub+relax_ub))
                tc.ub = tc.ub+relax_ub
                tc.lb = tc.lb-relax_lb
            prog = pt.to_rmpyl(tcs)
            prog.to_ptpn(filename='paris_stn_relaxation_posterior.tpn')
        else:
            print('Relaxation failed...')

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

def run_json_benchmark(bench_file):
    """
    Loads a given JSON benchmark file and runs it with PARIS.
    """
    benchmark = from_json_to_rmpyl_benchmark(bench_file)

    for test_name,tcs in benchmark.items():
        print('Running '+test_name)
        check_cc_strong_controllability(tcs,example_name=test_name)

def load_zipcar_test(tests_folder,test_name):
    """
    Loads and runs a specific strong controllability test from the Zipcar benchmarks.
    """
    folder = os.path.abspath(os.path.expanduser(tests_folder))
    tcs = load_pickle_file(os.path.join(folder,test_name))

    counts=[0,0,0]
    for tc in tcs:
        if tc.type=='controllable':
            counts[0]+=1
        elif tc.type=='uncontrollable_bounded':
            counts[1]+=1
        elif tc.type=='uncontrollable_probabilistic':
            counts[2]+=1

    check_cc_strong_controllability(tcs,example_name='stp_'+test_name)

    print('\t* Controllable durations %d'%(counts[0]))
    print('\t* Uncontrollable durations %d'%(counts[1]))
    print('\t* Probabilistic durations %d'%(counts[2]))

def run_all_zipcar_tests(results_file,optimize_partition,benchtype='json'):
    """
    Runs PARIS in all Zipcar tests.
    """
    global_start=time.time()
    if benchtype == 'json':
        benchmark = from_json_to_rmpyl_benchmark('../benchmarks/json/car_sharing.json')
    else:
        benchmark = load_pickle_file('../benchmarks/zipcar_tests/all_tests.pickle')

    pytemp={}
    for test,tcs in benchmark.items():

        #Counts the different types of constraints
        tc_count={'controllable':0,'uncontrollable_bounded':0,'uncontrollable_probabilistic':0}
        for tc in tcs:
            tc_count[tc.type]+=1
        print('\n##### Running test '+test+' containing')
        print('\t* Controllable durations %d'%(tc_count['controllable']))
        print('\t* Uncontrollable durations %d'%(tc_count['uncontrollable_bounded']))
        print('\t* Probabilistic durations %d'%(tc_count['uncontrollable_probabilistic']))

        prob_success,elapsed,risk_bound,risk_gap = check_cc_strong_controllability(tcs,
                                                                         example_name=test,
                                                                         write_tpns=False,
                                                                         gaussian_optimize_partition=optimize_partition)
        if prob_success>=0.0:
            pytemp[test]={'success':True,'risk':1.0-prob_success,'elapsed':elapsed,
                          'risk_bound':risk_bound,'risk_gap':risk_gap,'tc_count':tc_count}
        else:
            pytemp[test]={'success':False,'risk':-1,'elapsed':elapsed,
                          'risk_bound':risk_bound,'risk_gap':risk_gap,'tc_count':tc_count}

    with open(results_file,'wb') as f:
        print('Writing PARIS results to pickle file.')
        pickle.dump(pytemp,f)

    print('Finished everything in %.3f seconds.'%(time.time()-global_start))


if __name__=='__main__':

    if len(sys.argv)<2:
        print('Please choose an integer option between 0 and 3.')
        sys.exit(0)
    else:
        option = int(sys.argv[1])

    if option in [0,1]:
        if len(sys.argv)>=3 and sys.argv[2].endswith('pickle'):
            if option==0:
                print('Running Zipcar benchmarks WITHOUT partition optimization.')
                run_all_zipcar_tests(results_file=sys.argv[2],optimize_partition=False)
            else:
                print('Running Zipcar benchmarks WITH partition optimization.')
                run_all_zipcar_tests(results_file=sys.argv[2],optimize_partition=True)
        else:
            print('Please provide a valid pickle file name to store the benchmark results.')

    elif option == 2:
        if len(sys.argv)>=3 and sys.argv[2].endswith('pickle'):
            check_cc_strong_controllability_rmpyl(rmpyl_pickle=sys.argv[2])
        else:
            print('Please provide a valid pickle file containing PARIS results.')
    elif option==3:
        check_cc_strong_controllability(stnu_stedl(),example_name='stnu_stedl')
        check_cc_strong_controllability(stp_uniform_rubato(),example_name='stp_uniform_rubato')
        check_cc_strong_controllability(stp_rubato(),example_name='stp_rubato')
        check_cc_strong_controllability(stp_picard(),example_name='stp_picard')
    else:
        print('Option %d not available'%(option))
