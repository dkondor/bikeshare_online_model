/*  -*- C++ -*-
 * graph.h -- simple graph implementation storing it as a list of edges
 * 	includes implementation for finding connected components of a symmetric graph
 * 	and an implementation of finding a maximum matching for a bipartite graph
 * 		or DAG using the Hopcroft-Karp algorithm
 * 
 * Copyright 2018 Daniel Kondor <kondor.dani@gmail.com>
 * 
 * Hopcroft-Karp algorithm adapted from
 * http://www.geeksforgeeks.org/hopcroft-karp-algorithm-for-maximum-matching-set-2-implementation/
 * (no license provided for it)
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following disclaimer
 *   in the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of the  nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * 
 */
 
#ifndef GRAPH_H
#define GRAPH_H

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <unordered_map>
#include <algorithm>
#include <functional>
#include "tsv_iterator.h"
#include "iterator_zip.h"



/* 
 * main class storing a graph and implementing search for connected components
 * and maximum matching */
class graph {
	public:
		graph():nnodes(0),nedges(0),edges(0),outdeg(0),idx(0),nnodes_v(0),edges_size(0),nodes_size(0),edges_owned(true) {  }
		void clear() {
			if(edges_owned) if(edges) free(edges);
			edges_vect.clear(); edges = 0;
			if(outdeg) free(outdeg); if(idx) free(idx); outdeg = 0; idx = 0;
			nnodes = 0; nedges = 0; nnodes_v = 0; NIL = 0; edges_size = 0; nodes_size = 0;
		}
		~graph() { clear(); }
	
		
		/* There are four ways to create the graph, main difference is the format of input and thus
		 * the memory requirement.
		 * Storing the graph requires 4 bytes / edge + 12 bytes / node. The following functions are
		 * provided for creating the graph:
		 * 
		 * 	1. From a partitioned (sorted) input, supplied as forward iterators of std::pair<unsigned int,unsigned int>.
		 * 		In this case, all edges of a node in the input should be grouped together at least
		 * 		(or already sorted).
		 * 		Ideally, the iterators supplied need not belong to a container, but should be
		 * 		generated / read from a file on the fly. In this case, there is no additional
		 * 		memory requirement over storing the graph. */
		template<class it, class sent>
		int create_graph_partitioned(it&& e, const sent& end, std::unordered_map<unsigned int,unsigned int>* ids_map);
		 
		 /* 2. From general input supplied by random access iterators (to std::pair<unsigned int,unsigned int>).
		 * 		In this case, these iterators are used to sort the input. In this case, the additional
		 * 		memory requirement is that of the container of the iterators given, typically
		 * 		8 bytes / edge. */
		template<class it, class sent>
		int create_graph_in_place_sort(it&& e, const sent& end, std::unordered_map<unsigned int,unsigned int>* ids_map);
		 
		 /*  3. From general input supplied by a pair of forward iterators;
		 * 		in this case, all edges are copied into temporary arrays and sorted there, so again
		 * 		the data should be generated / read by the iterators on the fly;
		 * 		in this case, there is an additional 4 bytes / edge memory requirement during creating
		 * 		the graph */
		template<class it, class sent>
		int create_graph_copy_sort(it&& e, const sent& end, std::unordered_map<unsigned int,unsigned int>* ids_map);
		  
		 /*	4. from general input supplied as std::vectors / C arrays:
		 * 		edges should be supplied as two separate arrays or vector (i.e. edge i points from
		 * 		e1[i] -> e2[i]); this instance will take over the 2nd array (endpoints of the edges)
		 * 		in this case, there is no additional memory requirement (but creating those arrays
		 * 		will already have had 8 bytes / edge memory requirement, of which 4 bytes / edge can
		 * 		be freed after having created the graph */
		int create_graph_arrays(unsigned int* e1, unsigned int* e2, size_t e_size, std::unordered_map<unsigned int, unsigned int>* ids_map);
		int create_graph_vectors(std::vector<unsigned int>& e1, std::vector<unsigned int>& e2, std::unordered_map<unsigned int, unsigned int>* ids_map);
		
		
		/* Helper function for the above to handle reading the graph from a file.
		 * If partitioned == true, it expects already partitioned input; otherwise
		 * it sorts the input.
		 * If ids != null, it replaces ids in the edges to form a continuous
		 * range 0...N-1, and the real ids from the file are stored in the supplied
		 * vector */
		int read_graph(FILE* f, bool partitioned, std::vector<unsigned int>* ids);
		
		/* make the graph symmetric, i.e. for all a->b edges, make sure the reverse, b->a edge is present as well
		 * returns 0 on success, 1 on error */
		int make_symmetric();
		
		
		/* write graph to the given file; the user can specify a function to map
		 * the continuous ids (0...N-1) to any real node IDs (e.g. the mapping
		 * created when reading the graph) */
		void write_graph(FILE* f, std::function<unsigned int(unsigned int)> ids = [](const unsigned int& x) {return x;}) {
			if(!f) return;
			for(auto it = edges_begin(); it != edges_end(); ++it) fprintf(f,"%u\t%u\n",ids(it->first),ids(it->second));
		}
		/* same, but use a std::vector as the ID mapping */
		void write_graph(FILE* f, const std::vector<unsigned int>& ids) {
			write_graph(f,[ids](const unsigned int& x){return ids[x];});
		}
	
		/* iterator interface to access a group of edges (definitions below) */
		struct edges_iterator; struct edges_iterator_sentinel;
		edges_iterator edges_begin() const { return edges_iterator(this,0); }
		edges_iterator_sentinel edges_end() const { return edges_iterator_sentinel(); }
		edges_iterator edges_begin_n(unsigned int n_) const { return edges_iterator(this,n_,true); }
	
		/* find connected components in a symmetric graph
		 * NOTE: caller has to ensure that the graph is symmetric for this!
		 * results are stored in the vector provided, i.e. node i will be in SCC sccids[i] */
		unsigned int find_sccs(std::vector<unsigned int>& sccids) const;
		
		
		/* calculate maximum matching using the Hopcroft-Karp algorithm
		 * store the edges in the maximum matching in the provided vector (res)
		 * if use_r == false, use a version of dfs without recursion
		 * (using recursion might result in stack overflow even for
		 * moderate size graphs as well)
		 * 
		 * code adapted from
		 * http://www.geeksforgeeks.org/hopcroft-karp-algorithm-for-maximum-matching-set-2-implementation
		 */
		int maxmatch_hk(std::vector<std::pair<unsigned int, unsigned int> >& res, bool use_r = false) const;
	
		/* get memory used by this class + working memory required for HK graph matching */
		size_t get_memory_hk() const {
			size_t mem = edges_size*sizeof(unsigned int); /* for edges array */
			mem += nodes_size*sizeof(uint64_t); /* for idx array */
			mem += nodes_size*sizeof(unsigned int); /* for outdeg array */
			mem += sizeof(unsigned int)*(4*nnodes+nnodes_v+5); /* memory required for HK matching */
			return mem;
		}
		
		uint64_t num_edges() const { return nedges; }
		unsigned int num_nodes() const { return nnodes; }
		
		/* sort outgoing edges from each node according to "weights" given
		 * (edges with lower weights are given preference)
		 * this might affect how edges are selected for maximum matching
		 * ew(x,y) should return a weight for x->y edge
		 * x and y are IDs used by this class, so if there were replaced from
		 * original IDs, the caller should take that into account as well */
		template <class ew> void sort_out_edges(ew&& w);
		
	protected:
		unsigned int nnodes;
		uint64_t nedges;
		unsigned int* edges;
		unsigned int* outdeg;
		uint64_t* idx; // node i will have edges in [ edges[idx[i]] ; edges[idx[i]+outdeg[i]] ) -- note: outdeg could be omitted and only idx used, but this is more convenient
		unsigned int nnodes_v; // nodes in the v set (i.e. the maximum value + 1 of edges[i] for any i)
		unsigned int NIL; // maximum of nnodes_v and nnodes, used for bookeeping for maximum matching calculation
	
		const size_t EDGES_GROW = 33554432; // allocate memory for edges in 128MB chunks
		const size_t NODES_GROW = 262144; // allocate memory for nodes in 1M and 2M chunks
		
		std::vector<unsigned int> edges_vect; /* optionally, the edges are stored in an std::vector<int> received from
			the caller; in this case edges == edges_vect.data() and should not be freed later */
		bool edges_owned; /* this flag determines whether the edges array should be freed by the destructor */
		
		/* size of the edges and node arrays, and functions to grow them */	
		size_t edges_size;
		size_t nodes_size;
		
		int grow_nodes() {
			size_t new_size = nodes_size + NODES_GROW;
			unsigned int* tmp = (unsigned int*)realloc(outdeg,new_size*sizeof(unsigned int));
			if(!tmp) return 1;
			outdeg = tmp;
			uint64_t* tmp2 = (uint64_t*)realloc(idx,new_size*sizeof(uint64_t));
			if(!tmp2) return 1;
			idx = tmp2;
			nodes_size = new_size;
			return 0;
		}
		
		static int grow_edges_s(unsigned int** e, size_t* s, size_t grow) {
			size_t new_size = *s + grow;
			unsigned int* tmp = (unsigned int*)realloc(*e,new_size*sizeof(unsigned int));
			if(!tmp) return 1;
			*e = tmp;
			*s = new_size;
			return 0;
		}
		
		int grow_edges() {
			return grow_edges_s(&edges,&edges_size,EDGES_GROW);
		}
		
		/* common helper interface used internally doing all the work expecting sorted / partitioned input
		 * it and sent should be zip iterators pointing to the data or iterators of std::pair with the input.
		 * If ids_map is not null, ids are replaced and the map is filled with the mapping;
		 * otherwise, ids are allocated up to the maximum of the id found in the input */
		template<class it, class sent>
		int create_graph_sorted(it&& e, const sent& end, std::unordered_map<unsigned int,unsigned int>* ids_map, bool copy_out_edges);
	
		/* 3-4. common interface for creating the edges after copying the out-edges to the edges array and using a separate in-edges array */
		int create_graph(unsigned int* in_edges, std::unordered_map<unsigned int,unsigned int>* ids_map);
		
		
		/* helper functions for maximum matching */
		
		/* dfs for maximum matching
		 * Returns true if there is an augmenting path beginning with vertex u (non-recursive version) */
		void dfs_nor(unsigned int u, unsigned int* pairU, unsigned int* pairV, unsigned int* dist, unsigned int* path, uint64_t* ix) const;
		bool dfs(unsigned int u, unsigned int* pairU, unsigned int* pairV, unsigned int* dist) const;
		/* bfs for maximum matching
		 * Returns true if there is an augmenting path, else returns false */
		bool bfs(unsigned int* pairU, unsigned int* pairV, unsigned int* dist, unsigned int* Q) const;
	
		unsigned int real_deg(int n) const;
	
	public:
		/* iterator interface to process a group of edges
		 * this is stricly a forward iterator and it does not allow modification
		 * it could be extended to a random access iterator easily, but none of 
		 * the use cases requires it */
		struct edges_iterator_sentinel {
			/* separate empty class to use as the end of loops */
		};
		struct edges_iterator : std::iterator<std::input_iterator_tag, const std::pair<unsigned int,unsigned int> > {
			public:
				const std::pair<unsigned int,unsigned int>& operator *() const {
					if(is_end) handle_error("graph::edges_iterator used past the end!\n");
					return current;
				}
				const std::pair<unsigned int,unsigned int>* operator ->() const {
					if(is_end) handle_error("graph::edges_iterator used past the end!\n");
					return &current;
				}
				void operator ++() {
					if(is_end) return;
					i++;
					if(i >= g->nedges) { is_end = true; return; }
					current.second = g->edges[i];
					while(i >= g->idx[n] + g->outdeg[n]) {
						if(only_n) { is_end = true; return; }
						n++;
						if(n >= g->nnodes) handle_error("graph::edges_iterator: inconsistent graph structure!\n");
					}
					current.first = n;
				}
				edges_iterator(const graph* g_, unsigned int n_=0, bool only_n_ = false) {
					g = g_;
					i = 0;
					n = n_;
					is_end = false;
					only_n = only_n_;
					if(n >= g->nnodes) { is_end = true; return; }
					i = g->idx[n];
					while(i >= g->idx[n] + g->outdeg[n]) {
						if(only_n) { is_end = true; return; }
						n++;
						if(n >= g->nnodes) handle_error("graph::edges_iterator: inconsistent graph structure!\n");
					}
					current = std::make_pair(n,g->edges[i]);
				}
				bool operator ==(const edges_iterator& it) const {
					if(g != it.g) handle_error("graph::edges_iterator: comparison between different graphs attempted!\n");
					if(is_end != it.is_end) return false;
					if(is_end || i == it.i) return true;
					return false;
				}
				bool operator !=(const edges_iterator& it) const {
					return !(operator==(it));
				}
				bool operator ==(const edges_iterator_sentinel& it) const { return is_end; }
				bool operator !=(const edges_iterator_sentinel& it) const { return !is_end; }
			protected:
				std::pair<unsigned int,unsigned int> current;
				size_t i;
				unsigned int n;
				const graph* g;
				bool is_end;
				bool only_n;
		};		
};



/* templated functions for creating the graph */

template<class it, class sent>
int graph::create_graph_sorted(it&& e, const sent& end, std::unordered_map<unsigned int,unsigned int>* ids_map, bool copy_out_edges) {
	if(grow_nodes()) { fprintf(stderr,"graph::create_graph_sorted(): could not allocate memory!\n"); return 1; }
	if(copy_out_edges) if(grow_edges()) { fprintf(stderr,"graph::create_graph_sorted(): could not allocate memory!\n"); return 1; }
	if(ids_map) ids_map->clear();
	
	/* 2. count degrees, replace ids if needed */
	unsigned int last_id = 0;
	unsigned int deg = 0;
	unsigned int i = 0; // node index
	uint64_t j = 0; // edge index
	bool first = true;
	unsigned int max_out_edge = 0;
	
	for(;e != end;++e,j++) {
		/* note: potentially using zip_iterators, so cannot use e->first;
		 * but can use any other iterator returning an std::pair<unsigned int, unsigned int> */
		unsigned int id1 = (*e).first;
		unsigned int id2 = (*e).second;
		
		if(id1 != last_id || first) {
			if(!first) {
				outdeg[i] = deg;
				i++;
				if(i == nodes_size) if(grow_nodes()) { fprintf(stderr,"graph::create_graph_sorted(): could not allocate memory!\n"); return 1; }
			}
			first = false;
			
			idx[i] = j;
			if(ids_map) {
				auto it1 = ids_map->find(id1);
				if(it1 != ids_map->end()) {
					fprintf(stderr,"graph::create_graph_sorted(): edge origin %u appears in more than one place (input not paritioned)!\n",id1);
					return 1;
				}
				ids_map->insert(std::make_pair(id1,i));
			}
			else {
				if(id1 < last_id) {
					fprintf(stderr,"graph::create_graph_sorted(): input is not sorted at edge %u -- %u!\n",id1,id2);
					return 1;
				}
				while(i < id1) {
					outdeg[i] = 0;
					i++;
					if(i == nodes_size) if(grow_nodes()) { fprintf(stderr,"graph::create_graph_sorted(): could not allocate memory!\n"); return 1; }
					idx[i] = j;
				}
			}
			deg = 0;
			last_id = id1;
		}
		if(!ids_map) if(id2 > max_out_edge) max_out_edge = id2;
		deg++;
		if(copy_out_edges) {
			if(j == edges_size) if(grow_edges()) { fprintf(stderr,"graph::create_graph_sorted(): could not allocate memory!\n"); return 1; }
			edges[j] = id2;
		}
	}
	outdeg[i] = deg;
	
	nedges = j;
	
	/* replace ids in out edges if using ids_map */
	if(ids_map) {
		for(j=0;j<nedges;j++) {
			auto it2 = ids_map->find(edges[j]);
			if(it2 == ids_map->end()) {
				i++;
				if(i == nodes_size) if(grow_nodes()) { fprintf(stderr,"graph::create_graph_sorted(): could not allocate memory!\n"); return 1; }
				ids_map->insert(std::make_pair(edges[j],i));
				idx[i] = nedges;
				outdeg[i] = 0;
				edges[j] = i;
			}
			else edges[j] = it2->second;
		}
		i++;
	}
	else for(i++;i<max_out_edge;i++) {
		if(i >= nodes_size) if(grow_nodes()) { fprintf(stderr,"graph::create_graph_sorted(): could not allocate memory!\n"); return 1; }
		idx[i] = nedges;
		outdeg[i] = 0;
	}
	nnodes = i;
	nnodes_v = nnodes;
	NIL = nnodes_v;
	return 0;
}
		

template<class it, class sent>
int graph::create_graph_partitioned(it&& e, const sent& end, std::unordered_map<unsigned int,unsigned int>* ids_map) {
	clear();
	return create_graph_sorted(e,end,ids_map,true);
}
		

/* 2. Create graph with sorting the edges using the supplied iterators (which should be random access
 * 	iterators to std::pair<unsigned int, unsigned int>) */
template<class it, class sent>
int graph::create_graph_in_place_sort(it&& e, const sent& end, std::unordered_map<unsigned int,unsigned int>* ids_map) {
	clear();
	std::sort(e,end,[](const std::pair<unsigned int,unsigned int> a, const std::pair<unsigned int,unsigned int> b) {
			return a.first < b.first;
		});
	return create_graph_sorted(e,end,ids_map,true);
}

/* 3. create a graph from edges supplied as iterators of std::pair<unsigned int,unsigned int>
 * 
 * the iterators should be forward iterators, ideally generating the data on the fly, as it will be
 * copied into temporary arrays */
template<class it, class sent>
int graph::create_graph_copy_sort(it&& e, const sent& end, std::unordered_map<unsigned int,unsigned int>* ids_map) {
	clear();
	
	/* copy all data into the edges and temporary arrays */
	unsigned int* in_edges = 0;
	size_t in_edges_size = 0;
	size_t j = 0;
	int ret = 0;
	if( grow_edges() || grow_edges_s(&in_edges,&in_edges_size,EDGES_GROW) )
		{ fprintf(stderr,"graph::create_graph_copy_sort(): could not allocate memory!\n"); ret = 1; goto create_graph_copy_sort_err; }
	
	for(;e!=end;++e,j++) {
		if(j == edges_size) if(grow_edges()) { fprintf(stderr,"graph::create_graph_copy_sort(): could not allocate memory!\n"); ret = 1; goto create_graph_copy_sort_err; }
		if(j == in_edges_size) if(grow_edges_s(&in_edges,&in_edges_size,EDGES_GROW))
			{ fprintf(stderr,"graph::create_graph_copy_sort(): could not allocate memory!\n"); ret = 1; goto create_graph_copy_sort_err; }
		edges[j] = (*e).first;
		in_edges[j] = (*e).second;
	}
	nedges = j;
	ret = create_graph(in_edges, ids_map);
	
create_graph_copy_sort_err:
	if(in_edges) free(in_edges);
	return ret;
}		


template <class ew>
void graph::sort_out_edges(ew&& w) {
	for(unsigned int u = 0; u < nnodes; u++) if(outdeg[u] > 1) {
		unsigned int* start = edges + idx[u];
		unsigned int* end = edges + idx[u] + outdeg[u];
		std::sort(start,end,[&w,u](unsigned int x,unsigned int y) {
			return w(u,x) < w(u,y);
		});
	}
}


#endif

