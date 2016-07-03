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

Defines a temporal interface class.

@author: Pedro Santana (psantana@mit.edu).
"""
from .utils import to_rmpyl, from_rmpyl
try:
    from rmpyl.constraints import TemporalConstraint    
except ImportError:
    raise ImportError('pyTemporal currently uses the temporal constraints definitions from RMPyL.')

try:
    from . import networkx_interface as nxi; _NETWORKX_FOUND=True
except ImportError:
    _NETWORKX_FOUND=False; print('networkx not found.')

try:
    from . import gurobi_interface as gri; _GUROBI_FOUND=True
except ImportError:
    _GUROBI_FOUND=False; print('Gurobi not found.')

try:
    from . import cplex_interface as cpi; _CPLEX_FOUND=True
except Exception:
    _CPLEX_FOUND=False; print('CPLEX not found.')


class PyTemporal(object):
    """
    Class used to interface with the pyTemporal package. Used to abstract the
    internal implementation of the various temporal reasoning functions.
    """
    def __init__(self,stn_consistency='gurobi',stn_relaxation='gurobi',stp_sc='gurobi'):

        self._tcs = set() #Set of temporal constraints

        if not stn_consistency in [None,'networkx','gurobi','cplex']:
            raise ValueError('STN consistency checkers: networkx, gurobi, cplex, or None.')
        if not stn_relaxation in [None,'gurobi','cplex']:
            raise ValueError('STN relaxation: gurobi, cplex, or None.')

        self._use_networkx=False; self._use_gurobi=False; self._use_cplex=False
        if stn_consistency=='networkx':
            if _NETWORKX_FOUND:
                self._nxi = nxi.NetworkxInterface()
            else:
                raise ImportError('NetworkX cannot be used')
            self._use_networkx = True
            self.consistent_stn = self._nxi.consistent_stn

        if stn_consistency=='gurobi' or stn_relaxation=='gurobi':
            if _GUROBI_FOUND:
                self._gri = gri.GurobiInterface()
            else:
                raise ImportError('Gurobi cannot be used')
            self._use_gurobi=True
            if stn_consistency=='gurobi':
                self.consistent_stn = self._gri.consistent_stn
            if stn_relaxation=='gurobi':
                self.stn_relaxation = self._gri.stn_relaxation
            # if stp_sc=='gurobi':
            #     self.strongly_controllable_stp=self._gri.strongly_controllable_stp

        if stn_consistency=='cplex' or stn_relaxation=='cplex':
            if _CPLEX_FOUND:
                self._cpi = cpi.CplexInterface()
            else:
                raise ImportError('CPLEX cannot be used')
            self._use_cplex=True
            if stn_consistency=='cplex':
                self.consistent_stn = self._cpi.consistent_stn
            if stn_relaxation=='cplex':
                self.stn_relaxation = self._cpi.stn_relaxation


    @property
    def temporal_constraints(self):
        """
        Returns the set of temporal constraints that are currently part of the
        interface.
        """
        return self._tcs

    def add_temporal_constraint(self,tc):
        """
        Adds a temporal constraint to the interface.
        """
        if isinstance(tc,TemporalConstraint):
            self._tcs.add(tc)
            if self._use_networkx:
                self._nxi.add_temporal_constraint(tc)
            if self._use_gurobi:
                self._gri.add_temporal_constraint(tc)
            if self._use_cplex:
                self._cpi.add_temporal_constraint(tc)
        else:
            raise TypeError('Temporal constraints must be RMPyL TemporalConstraint types.')

    def remove_temporal_constraint(self,tc):
        """
        Removes a temporal constraint from the interface.
        """
        if isinstance(tc,TemporalConstraint):
            self._tcs.discard(tc)
            if self._use_networkx:
                self._nxi.remove_temporal_constraint(tc)
            if self._use_gurobi:
                self._gri.remove_temporal_constraint(tc)
            if self._use_cplex:
                self._cpi.remove_temporal_constraint(tc)
        else:
            raise TypeError('Temporal constraints must be RMPyL TemporalConstraint types.')

    def to_rmpyl(self,tcs=None):
        """
        Converts the temporal constraints into an RMPyL program, which can then
        be exported to a TPN.
        """
        if tcs==None:
            return to_rmpyl(self.temporal_constraints)
        else:
            return to_rmpyl(tcs)

    def from_rmpyl(self,prog):
        """
        Imports the temporal constraints from an RMPyL program.
        """
        tcs = from_rmpyl(prog)
        for tc in tcs:
            self.add_temporal_constraint(tc)
