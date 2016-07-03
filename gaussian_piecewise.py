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

Tools for piecewise-linear approximations of univariate, Gaussian cdfs.

@author: Pedro Santana (psantana@mit.edu).
"""
import numpy as np
import numpy.linalg as la
import scipy.stats as st
from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
import time

def get_gaussian_partition(mean,var,num_div,lb_sigma=None,ub_sigma=None,
                           optimize_partition=False,random_part_init=False,
                           lr=0.05,tol=1e-5,max_iter=10000):
    """
    Returns two lists, lb_breaks and ub_breaks, of partition breakpoints for the
    piecewise-linear approximation of the lower and upper segments, respectively,
    of a Gaussian distribution.
    """
    stdev = np.sqrt(var)

    #If the upper or lower cutoffs have been specified (in units of standard
    #deviation), uses them. Otherwise, uses the number of divisions.
    part_min = max(0.0,mean-stdev*lb_sigma if lb_sigma!=None else mean-num_div*stdev)
    part_max = mean+stdev*ub_sigma if ub_sigma!=None else mean+num_div*stdev

    step_lb = float(mean-part_min)/num_div
    step_ub = float(part_max-mean)/num_div

    #Default partition breaks
    lb_breaks = [mean+step_lb*i for i in range(-num_div,1)]
    ub_breaks = [mean+step_ub*i for i in range(0,num_div+1)]

    if optimize_partition:
        if random_part_init:
            part = []
            #Adds random intermediate partition points sampled from the same
            #distribution
            added_ub=0; added_lb=0
            while len(part)<(2*num_div-2):
                new_p = st.norm.rvs(loc=mean,scale=stdev)

                if (added_lb<num_div-1) and new_p>part_min and new_p<mean:
                    added_lb+=1
                    part.append(new_p)

                if (added_ub<num_div-1) and new_p>mean and new_p<part_max:
                    added_ub+=1
                    part.append(new_p)

            #Adds the extrema, the mean, and sorts the partition vector
            part = part+[part_min,part_max,mean]
            part.sort()
        else:
            #Simply removes the repeated mean element
            part = lb_breaks+ub_breaks[1:]
            #part = [mean+i*stdev for i in range(-num_div,num_div+1) if mean+i*stdev >= 0.0]

        opt_part = list(optimize_piecewise_gaussian_approximation(part,mean,var,lr,tol,max_iter,True))
        mean_idx = opt_part.index(mean)
        lb_breaks = opt_part[:mean_idx+1]; ub_breaks = opt_part[mean_idx:]

    return lb_breaks,ub_breaks


def optimize_piecewise_gaussian_approximation(X,mean,var,u=0.05,tol=1e-5,
                                              max_iter=10000,verbose=True):
    """
    Optimizes the piecewise linear approximation of Gaussian cdf by improving
    the piecewise constant upper bound on its pdf. It does so using Gradient
    Descent.
    """
    start_time = time.time()
    mean_idx = X.index(mean)

    Xt = np.array(X)
    grad = _piecese_gaussian_gradient(Xt,mean,var,mean_idx)
    grad_norm = la.norm(grad)

    start_gap = piecewise_gaussian_bound_gap(Xt,mean,var)

    iter_count=1
    while (grad_norm>=tol) and (iter_count<=max_iter):
        Xt = _partition_projection(Xt-u*grad,mean_idx)
        grad = _piecese_gaussian_gradient(Xt,mean,var,mean_idx)
        grad_norm = la.norm(grad)
        iter_count+=1

    end_gap = piecewise_gaussian_bound_gap(Xt,mean,var)

    if _valid_partition(Xt):
        if verbose:
            print('\nElapsed time: %.3f ms'%((time.time()-start_time)*1000.0))
            print('Distribution: N(%f,%f)'%(mean,var))
            print('Initial partitions: '+' ,'.join(['%.3f'%n for n in X]))
            print('Final partitions: '+' ,'.join(['%.3f'%n for n in Xt]))
            print('Final gradient norm: %f'%grad_norm)
            if iter_count>max_iter:
                print('WARNING: Maximum number of iterations (%d) exceeded'%max_iter)
            else:
                print('GD converged in %d iterations.'%iter_count)
            if end_gap >start_gap:
                print('ERROR: probability gap deteriorated from %f to %f'%(start_gap,end_gap))
                return X
            else:
                print('Start piecewise gap: %f '%(start_gap))
                print('End piecewise gap: %f '%(end_gap))
                print('Gap improvement: %f (%f %% relative to start gap)'%(start_gap-end_gap,(1.0-end_gap/start_gap)*100.0))
                return Xt
    else:
        raise RuntimeError('Invalid partition found.')


def plot_piecewise_gaussian_approximation(X,mean,var):
    """Plots a piecewise-constant approximation of a Gaussian pdf."""
    x = np.linspace(X[0],X[-1],100)
    y_gauss = st.norm.pdf(x,loc=mean,scale=np.sqrt(var))

    f = plt.figure()
    ax = f.add_subplot(1,1,1)
    ax.hold(True)
    for i in range(len(X)-1):
        width = X[i+1]-X[i]
        h = X[i] if X[i]>=mean else X[i+1]
        height = st.norm.pdf(h,loc=mean,scale=np.sqrt(var))
        ax.add_patch(Rectangle((X[i],0.0),width,height,facecolor='blue',alpha=0.3))
    ax.plot(x,y_gauss,'b',linewidth=3)
    #ax.fill_between(x,y_gauss,facecolor='white',alpha=0.5)

    return f,ax

def piecewise_gaussian_bound_gap(X,mean,var):
    """
    (Positive) Gap between the piecewise-linear approximation of the Gaussian cdf
    and the true cdf.
    """
    stdev = np.sqrt(var)
    true_prob = st.norm.cdf(X[-1],loc=mean,scale=stdev)-st.norm.cdf(X[0],loc=mean,scale=stdev)
    return piecewise_gaussian_bound(X,mean,var)-true_prob

def piecewise_gaussian_bound(X,mean,var):
    """Gradient vector of piecewise linear upper bound of a Gaussian cdf."""
    f = st.norm.pdf(X,loc=mean,scale=np.sqrt(var))
    g=0.0
    for i in range(1,len(X)):
        g+= (X[i]-X[i-1])*f[i] if X[i] <= mean else (X[i]-X[i-1])*f[i-1]
    return g

def _piecese_gaussian_gradient(X,mean,var,mean_idx):
    """Gradient vector of piecewise linear upper bound of a Gaussian cdf."""
    grad = np.zeros(X.shape)
    f = st.norm.pdf(X,loc=mean,scale=np.sqrt(var))
    for k in range(1,len(grad)-1):
        if k < mean_idx:#Left of mode#Not the mode, which is fixed
            grad[k]= (f[k]*(mean-X[k])/var)*(X[k]-X[k-1])+f[k]-f[k+1]
        elif k > mean_idx :  #Right of mode
            grad[k]= (f[k]*(mean-X[k])/var)*(X[k+1]-X[k])+f[k-1]-f[k]

        # if k!= mean_idx:# not np.isclose(X[k],mean): #Not the mode, which is fixed
        #     if X[k]<mean: #Left of mode
        #         grad[k]= (f[k]*(mean-X[k])/var)*(X[k]-X[k-1])+f[k]-f[k+1]
        #     else:  #Right of mode
        #         grad[k]= (f[k]*(mean-X[k])/var)*(X[k+1]-X[k])+f[k-1]-f[k]
    return grad

def _partition_projection(X,mean_idx):
    """
    Projects the partition vector onto the feasible region, where pi+1>=pi
    """
    if not _valid_partition(X):
        X_new = np.copy(X)

        #Saturates the partitions between the minimums and maximums in the
        #interval
        for s in [0,mean_idx]:
            for i in range(1,mean_idx):
                X_new[s+i] = max(X[s],min(X[s+mean_idx],X_new[s+i]))

        #Sorts the values (no guarantees that this is the optimal projection)
        for s in [0,mean_idx]:
            X_new[s+1:s+mean_idx] = np.sort(X_new[s+1:s+mean_idx])

        # for s in [0,mean_idx]:
        #     for i in range(1,mean_idx):
        #         if X_new[s+i]>X_new[s+i+1]:
        #             mean = (X_new[s+i]+X_new[s+i+1])/2.0
        #             X_new[s+i] = mean-1e-4
        #             X_new[s+i+1] = mean+1e-4

        # if not _valid_partition(X_new):
        #     import ipdb; ipdb.set_trace()
    else:
        X_new = X

        # for i in range(mean_idx+1,len(X)-1):
        #     X[i] = np.median(X[i-1:i+2])
    return X_new

def _valid_partition(X):
    for i in range(1,len(X)):
        if X[i]<X[i-1]:
            return False
    return True
