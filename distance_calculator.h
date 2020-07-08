/*  -*- C++ -*-
 * distance_calculator.h -- helper to calculate distances
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
 */

#ifndef DISTANCE_CALCULATOR_H
#define DISTANCE_CALCULATOR_H
#include "spdist.h"

#include <unordered_map>
#include <vector>
#include <set>

#include <stdint.h>



/* struct for calculating shortest path distances along a given network */
struct node_distances {
	struct edge_info {
		double d; /* edge distance */
		bool is_improved; /* flag if the edge is part of the improved network */
		explicit edge_info(double d_) : d(d_), is_improved(false) { }
		edge_info() : d(0), is_improved(false) { }
	};
	std::unordered_map<uint64_t,std::unordered_map<uint64_t,edge_info> > n;
	double improved_edge_weight;
	node_distances():improved_edge_weight(1.0) { }
	
	bool read_network(read_table2&& rt) {
		while(rt.read_line()) {
			uint64_t n1,n2;
			double d;
			if(!rt.read(n1,n2,d)) break;
			n[n1][n2] = edge_info(d);
			n[n2][n1] = edge_info(d);
		}
		if(rt.get_last_error() != T_EOF) {
			fprintf(stderr,"Error reading network:\n");
			rt.write_error(stderr);
			return false;
		}
		return true;
	}
	
	bool read_improved(read_table2&& rt, double improved_edge_weight_) {
		improved_edge_weight = improved_edge_weight_;
		unsigned int cnt = 0;
		while(rt.read_line()) {
			uint64_t n1,n2;
			if(!rt.read(n1,n2)) break;
			if(n.count(n1) == 0 || n.count(n2) == 0) {
				fprintf(stderr,"Improved edge %lu -- %lu not in network!\n",n1,n2);
				return false;
			}
			if(n[n1].count(n2) == 0 || n[n2].count(n1) == 0) {
				fprintf(stderr,"Improved edge %lu -- %lu not in network!\n",n1,n2);
				return false;
			}
			n[n1][n2].is_improved = true;
			n[n2][n1].is_improved = true;
			cnt++;
		}
		if(rt.get_last_error() != T_EOF) {
			fprintf(stderr,"Error reading improved edges:\n");
			rt.write_error(stderr);
			return false;
		}
		fprintf(stderr,"%u improved edges read\n",cnt);
		return true;
	}
	
	
	struct node {
		double d; /* current estimate of distance to this node */
		double real_d; /* "real" distance; the above can be the weighted distance */
		uint64_t node_id; /* node id */
		bool operator < (const node& n) const {
			/* note: node_id is part of the comparison so that nodes can be found exactly
			 * (even if distances are the same for multiple nodes)
			 * real_d is not part, we don't care about it when searching */
			return d < n.d || (d == n.d && node_id < n.node_id);
		}
	};
	
	struct sentinel {
/*		bool operator == (const iterator& it) const { return it.is_end(); }
		bool operator != (const iterator& it) const { return !it.is_end(); } */
	};
	
	struct iterator {
		const node_distances* nd;
		std::set<node_distances::node> q; /* queue of nodes to process by distance */
		std::unordered_map<uint64_t,double> nodes_seen;
		
		iterator(const node_distances* nd_, uint64_t start_node):nd(nd_) {
			q.insert(node{0.0,0.0,start_node});
		}
		iterator():nd(0) { }
		
		/* operators to copy change iterator -- latter only works if the same network is used */
		iterator(const iterator& it):nd(it.nd),q(it.q),nodes_seen(it.nodes_seen) { }
		iterator(iterator&& it):nd(it.nd),q(std::move(it.q)),nodes_seen(std::move(it.nodes_seen)) { }
		iterator& operator = (const iterator& it) {
			nd = it.nd;
			q = it.q;
			nodes_seen = it.nodes_seen;
			return *this;
		}
		iterator& operator = (iterator&& it) {
			nd = it.nd;
			q = std::move(it.q);
			nodes_seen = std::move(it.nodes_seen);
			return *this;
		}
		
		/* note: it is UB to dereference this iterator after the end */
		const node& operator* () const { return *q.begin(); }
		const node* operator -> () const { return &(*q.begin()); }
		bool is_end() const { return q.empty(); }
		bool operator == (const sentinel& s) const { return is_end(); }
		bool operator != (const sentinel& s) const { return !is_end(); }
		
		iterator& operator ++ () {
			step();
			return *this;
		}
		iterator operator ++(int) {
			iterator it(*this);
			step();
			return it;
		}
			
		void step() {
			auto it = q.begin();
			uint64_t current = it->node_id;
			double d = it->d;
			double real_d = it->real_d;
			q.erase(it);
			nodes_seen.insert(std::make_pair(current,0.0));
			
			/* add to the queue the nodes reachable from the current */
			for(const auto& x : nd->n.at(current) ) {
				uint64_t n1 = x.first; /* node ID */
				double real_d1 = real_d + x.second.d; /* total real distance this way */
				double d1 = d; /* total weighted distance this way */
				if(x.second.is_improved) d1 += x.second.d / nd->improved_edge_weight;
				else d1 += x.second.d;
				
				auto it3 = nodes_seen.find(n1);
				if(it3 == nodes_seen.end()) { /* this node was not seen yet, we can add to the queue */
					nodes_seen.insert(std::make_pair(n1,d1));
					q.insert(node{d1,real_d1,n1});
				}
				else {
					/* node already found, may need to be updated -- only if new distance is shorter */
					if(d1 < it3->second) {
						if(q.erase(node{it3->second,0,n1}) != 1) {
							fprintf(stderr,"Error: node %lu not found in the queue (distance: %f)!\n",n1,it3->second);
							throw std::runtime_error("node_distances::iterator::step(): invalid node queue detected!\n");
						}
						q.insert(node{d1,real_d1,n1});
						it3->second = d1;
					}
				}
			}
		}
	};
	
	typedef iterator distance_iterator;
	
	iterator begin(uint64_t start_node) const { return iterator(this,start_node); }
	sentinel end() const { return sentinel(); }
	sentinel end(uint64_t start_node) const { return sentinel(); }
	
	bool get_dist(uint64_t start_node, uint64_t end_node, double& d) const {
		throw std::runtime_error("node_distances::get_dist() is not implemented!\n");
	}
	
	double get_dist(uint64_t start_node, uint64_t end_node) const {
		throw std::runtime_error("node_distances::get_dist() is not implemented!\n");
	}
};


/* adapter for spatial index that takes care of remapping IDs */
template<class ixtype>
struct spindex_adapter : public sp_sorted_index<ixtype, true> {
	std::vector<uint64_t> mapping_reverse;
	std::unordered_map<uint64_t, unsigned int> mapping;
	
	bool read_mapping(const char* fn) {
		mapping_reverse.clear();
		mapping.clear();
		read_table2 rt(fn);
		while(rt.read_line()) {
			uint64_t id;
			if(!rt.read(id)) break;
			unsigned int ix = mapping_reverse.size();
			mapping_reverse.push_back(id);
			if(!mapping.insert(std::make_pair(id,ix)).second) {
				fprintf(stderr,"Duplicate ID (%lu) on input line %lu!\n",id,rt.get_line());
				return false;
			}
		}
		if(rt.get_last_error() != T_EOF) {
			fprintf(stderr,"Error reading IDs!\n");
			rt.write_error(stderr);
			return false;
		}
		if(mapping_reverse.size() != this->size()) {
			fprintf(stderr,"Number of IDs read (%lu) does not match expected number (%u)!\n",mapping_reverse.size(),this->size());
			return false;
		}
		return true;
	}
	
	struct const_iterator {
		protected:
			typedef typename sp_sorted_index<ixtype, true>::ixtype2 ixtype2;
			const spindex_adapter* a;
			const ixtype2* i;
			const_iterator(const spindex_adapter* a_, const ixtype2* i_):a(a_),i(i_) { }
		public:
			friend struct spindex_adapter;
			const_iterator():a(0),i(0) { }
			uint64_t operator * () const { return a->mapping_reverse.at(*i); }
			//~ uint64_t operator [] (size_t x) const { return a->mapping_reverse.at(i[x]); }
			bool operator == (const const_iterator& it) const { return i == it.i; }
			bool operator != (const const_iterator& it) const { return i != it.i; }
			bool operator <= (const const_iterator& it) const { return i <= it.i; }
			bool operator >= (const const_iterator& it) const { return i >= it.i; }
			bool operator < (const const_iterator& it) const { return i < it.i; }
			bool operator > (const const_iterator& it) const { return i > it.i; }
			
			const_iterator& operator ++ () {
				++i;
				return *this;
			}
			const_iterator operator ++(int) {
				const_iterator it(*this);
				++i;
				return it;
			}
			const_iterator& operator += (ssize_t x) { i += x; return *this; }
			const_iterator& operator -= (ssize_t x) { i -= x; return *this; }
			const_iterator operator + (ssize_t x) const {
				const_iterator it(*this);
				it += x;
				return it;
			}
			const_iterator operator - (ssize_t x) const {
				const_iterator it(*this);
				it -= x;
				return it;
			}
			ssize_t operator - (const const_iterator& it) const {
				return i - it.i;
			}
	};
	/* special iterator "wrapper" that also supplies the distances */
	struct distance_iterator : public const_iterator {
		public:
			struct current_t {
				ixtype d;
				uint64_t node_id;
				current_t():d(0),node_id(0) { }
			};
		protected:
			typedef typename sp_sorted_index<ixtype, true>::ixtype2 ixtype2;
			current_t current;
			unsigned int start_node;
			distance_iterator(const spindex_adapter* a_, const ixtype2* i_,
					unsigned int start):const_iterator(a_,i_), start_node(start) {
				if(a_ && i_) update_current();
			}
			
			void update_current() {
				current.node_id = this->a->mapping_reverse.at(*this->i);
				current.d = this->a->get_dist_ix(start_node, *this->i);
			}
		public:
			distance_iterator():start_node(0) { }
			friend class spindex_adapter;
			const current_t* operator -> () { return &current; }
			const current_t& operator * () { return current; }
			
			distance_iterator& operator ++ () { ++this->i; update_current(); return *this; }
			distance_iterator operator ++(int) {
				distance_iterator it(*this);
				++this->i;
				update_current();
				return it;
			}
			
			distance_iterator& operator += (ssize_t x) { this->i += x; update_current(); return *this; }
			distance_iterator& operator -= (ssize_t x) { this->i -= x; update_current(); return *this; }
			distance_iterator operator + (ssize_t x) const {
				distance_iterator it(*this);
				it += x;
				return it;
			}
			distance_iterator operator - (ssize_t x) const {
				distance_iterator it(*this);
				it -= x;
				return it;
			}
			
	};
	
	
	bool get_dist(uint64_t start, uint64_t end, ixtype& dist) const {
		return this->get_dist_ix(mapping.at(start), mapping.at(end), dist);
	}
	ixtype get_dist(uint64_t start, uint64_t end) const {
		return this->get_dist_ix(mapping.at(start), mapping.at(end));
	}
	/*
	const_iterator begin(uint64_t start_node) const {
		return const_iterator(this, this->begin(mapping.at(start_node)));
	}
	const_iterator end(uint64_t start_node) const {
		return const_iterator(this, this->end(mapping.at(start_node)));
	}
	*/
	distance_iterator begin(uint64_t start_node) const {
		unsigned int start2 = mapping.at(start_node);
		return distance_iterator(this, sp_sorted_index<ixtype, true>::begin(start2), start2);
	}
	distance_iterator end(uint64_t start_node) const {
		unsigned int start2 = mapping.at(start_node);
		return distance_iterator(this, sp_sorted_index<ixtype, true>::end(start2), start2);
	}
	typedef distance_iterator sentinel;
};


#endif



