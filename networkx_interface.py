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

Interface functions with the Networkx solver, which is mainly used to detect
inconsistent STN's using the Bellman-Ford algorithm.

@author: Pedro Santana (psantana@mit.edu).
"""
import networkx as nx

class NetworkxInterface(object):
    """
    Interface with Networkx for tasks related to temporal problems.
    """
    def __init__(self):
        self._dist_graph = nx.DiGraph() #Distance graph

    def add_temporal_constraint(self,tc):
        """
        Adds a temporal constraint to the Networkx object representing the 
        distance graph.
        """
        self._dist_graph.add_edge(tc.start,tc.end,weight=tc.ub)
        self._dist_graph.add_edge(tc.end,tc.start,weight=-tc.lb)        

    def remove_temporal_constraint(self,tc):
        """
        Removes a temporal constraint from the Networkx object representing the 
        distance graph.
        """
        self._dist_graph.remove_edge(tc.start,tc.end)
        self._dist_graph.remove_edge(tc.end,tc.start)        

    def consistent_stn(self,add_tcs=[]):
        """
        Returns whether a Simple Temporal Network is consistent.
        """
        #Checks if there are additional constraints to be temporarily added
        #to the graph.
        if len(add_tcs)>0:
            for tc in add_tcs: #Adds temporary constraints
                self.add_temporal_constraint(tc)
            #Checks if it is consistent
            neg_cycles = nx.negative_edge_cycle(self._dist_graph)

            for tc in add_tcs: #Removes temporary constraints
                self.remove_temporal_constraint(tc)          

            return not neg_cycles                               
        else:
            return not nx.negative_edge_cycle(self._dist_graph)
    