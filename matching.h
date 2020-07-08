/*  -*- C++ -*-
 * matching.h -- helper functions to calculate maximum matching
 * 
 * Copyright 2020 Daniel Kondor <kondor.dani@gmail.com>
 * 
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * You should have received a copy of the GNU Affero General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>
 * 
 * 
 */


#ifndef MATCHING_H
#define MATCHING_H

#include "graph.h"
#include <lemon/smart_graph.h>
#include <lemon/matching.h>

#include <vector>
#include <unordered_map>

#include <stdio.h>


namespace matching_helper {

	struct edge {
		unsigned int first;
		unsigned int second;
		unsigned int dist;
		edge() : first(0),second(0),dist(0) { } /* default constructor sets elements to avoid undefined behavior */
		edge(unsigned int first_, unsigned int second_, unsigned int dist_):first(first_),second(second_),dist(dist_) { }
		operator std::pair<unsigned int,unsigned int>() const { return std::make_pair(first,second); }
	};
	
	
	template<class It, class Sentinel>
	void calculate_matching(It&& it, const Sentinel& sent, std::vector<edge>& res, unsigned int max_wait_time, FILE* fout = 0) {
		res.clear();
		
		lemon::SmartBpGraph g;
		lemon::SmartBpGraph::RedNodeMap<unsigned int> ids_out(g);
		lemon::SmartBpGraph::BlueNodeMap<unsigned int> ids_in(g);
		lemon::SmartBpGraph::EdgeMap<unsigned int> w(g);
		
		{
			std::unordered_map<unsigned int,lemon::SmartBpGraph::RedNode> ids_out_map;
			std::unordered_map<unsigned int,lemon::SmartBpGraph::BlueNode> ids_in_map;
			
			for(;it != sent;++it) {
				unsigned int e1, e2, w1;
				{
					const edge& tmp = *it;
					e1 = tmp.first;
					e2 = tmp.second;
					w1 = tmp.dist;
				}
				if(w1 > max_wait_time) continue;
				w1 = max_wait_time - w1;
				
				/* n1 -> n2 edge */
				lemon::SmartBpGraph::RedNode n1;
				lemon::SmartBpGraph::BlueNode n2;
				
				auto it1 = ids_out_map.find(e1);
				if(it1 != ids_out_map.end()) n1 = it1->second;
				else {
					n1 = g.addRedNode();
					ids_out_map.insert(std::make_pair(e1,n1));
					ids_out[n1] = e1;
				}
				
				auto it2 = ids_in_map.find(e2);
				if(it2 != ids_in_map.end()) n2 = it2->second;
				else {
					n2 = g.addBlueNode();
					ids_in_map.insert(std::make_pair(e2,n2));
					ids_in[n2] = e2;
				}
				
				
				lemon::SmartBpGraph::Edge e = g.addEdge(n1,n2);
				w[e] = w1;
			}
			
			if(fout) fprintf(fout,"graph with %lu + %lu nodes and %d edges, ",ids_out_map.size(),ids_in_map.size(),g.edgeNum());
		}
		
		lemon::MaxWeightedMatching<lemon::SmartBpGraph, lemon::SmartBpGraph::EdgeMap<unsigned int> > matching(g,w);
		matching.run();
		if(fout) fprintf(fout,"matching has %d edges\n",matching.matchingSize());
		
		for(lemon::SmartBpGraph::RedNodeIt n(g); n != lemon::INVALID; ++n) {
			lemon::SmartBpGraph::BlueNode m = g.asBlueNode(matching.mate(n));
			if(m != lemon::INVALID) res.push_back(edge(ids_out[n],ids_in[m],max_wait_time - w[ matching.matching(n) ]));
		}
	}
	
	
	
	template<class It, class Sentinel>
	void calculate_matching_unweighted(It&& it, const Sentinel& sent, std::vector<std::pair<unsigned int, unsigned int> >& res, FILE* fout = 0) {
		res.clear();
		
		graph g;
		std::vector<unsigned int> ids_reverse_map;
		
		{
			std::unordered_map<unsigned int, unsigned int> ids_map;
			g.create_graph_partitioned(it, sent, &ids_map);
			
			if(fout) fprintf(fout,"graph with %lu nodes and %lu edges, ",ids_map.size(),g.num_edges());
			ids_reverse_map.resize(ids_map.size());
			for(const auto& x : ids_map) ids_reverse_map.at(x.second) = x.first;
		}
		
		if(g.maxmatch_hk(res, false)) throw std::runtime_error("Error calculating maximum matching!\n");
		/* replace IDs in result */
		for(auto& x : res) {
			x.first = ids_reverse_map[x.first];
			x.second = ids_reverse_map[x.second];
		}
	}
}


#endif

