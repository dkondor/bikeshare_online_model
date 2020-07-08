/*
 * sp_distance_matrix_p.cpp -- read path network, calculate
 * 	shortest paths, save in binary format
 * 
 * parallel version
 * 
 * Copyright 2020 Daniel Kondor <kondor.dani@gmail.com>
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


#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <algorithm>
#include <utility>
#include <limits>
#include <strings.h>
#include <atomic>
#include <thread>

#include "read_table.h"
#include "mmap.h"


struct node {
	double d; /* current estimate of distance to this node */
	//~ double real_d; /* "real" distance; the above can be the weighted distance */
	uint64_t node_id; /* node id */
	uint64_t ancestor; /* last node in the path leading to this */
	bool operator < (const node& n) const {
		/* note: node_id is part of the comparison so that nodes can be found exactly
		 * (even if distances are the same for multiple nodes);
		 * ancestor and real distance are not part of the comparison as that is not known or relevant when searching */
		return d < n.d || (d == n.d && node_id < n.node_id);
	}
};

template<class T>
class distance_worker {
	public:
		struct distance_worker_data {
			FileMappingT<T>& distance_file;
			FileMappingT<uint32_t>& index_file;
			const std::unordered_map<uint64_t,std::unordered_map<uint64_t,double > >& n;
			const std::vector<uint64_t>& node_ids_reverse;
			std::atomic<uint64_t> nodes_completed;
			
			distance_worker_data(FileMappingT<T>& distance_file_,
				FileMappingT<uint32_t>& index_file_,
				const std::unordered_map<uint64_t,std::unordered_map<uint64_t,double > >& n_,
				const std::vector<uint64_t>& node_ids_reverse_) :
					distance_file(distance_file_),
					index_file(index_file_),
					n(n_),
					node_ids_reverse(node_ids_reverse_),
					nodes_completed(0UL) { }
		};
	protected:
		distance_worker_data& d;
		const size_t thread_id;
		const size_t nthreads;
		const size_t nnodes;
		/* node distances in matrix order */
		std::vector<double> node_distances2;
		/* indexes to node_distances2 for sorting */
		std::vector<uint32_t> node_sorted_ix;
		std::set<node> q; /* queue of nodes to process by distance */
		std::unordered_map<uint64_t,double> node_distances; /* distance of all nodes from the start node */
		bool error;
		
		distance_worker() { }
		
		/* calculate all distances starting from node i */
		bool calculate_one(size_t i) {
			const std::unordered_map<uint64_t,std::unordered_map<uint64_t,double > >& n = d.n;
			const std::vector<uint64_t>& node_ids_reverse = d.node_ids_reverse;
			bool use_index = d.index_file.is_mapped();
			
			/* perform a search from the given node */
			uint64_t start_node = node_ids_reverse[i];
			
			node_distances[start_node] = 0;
			q.insert(node {0.0,start_node,start_node});
			
			do {
				auto it = q.begin();
				uint64_t current = it->node_id;
				double d = it->d;
				//~ double real_d = it->real_d;
				q.erase(it);
				
				/* add to the queue the nodes reachable from the current */
				for(const auto& x : n.at(current)) {
					uint64_t n1 = x.first; /* node ID */
					//~ double real_d1 = real_d + x.second.d; /* total real distance this way */
					double d1 = d; /* total weighted distance this way */
					//~ if(x.second.is_improved) d1 += x.second.d / improved_edge_weight;
					//~ else 
					d1 += x.second;
					auto it3 = node_distances.find(n1);
					if(it3 == node_distances.end()) { /* this node was not seen yet, we can add to the queue */
						node_distances.insert(std::make_pair(n1,d1));
						q.insert(node{d1,n1,current});
					}
					else {
						/* node already found, may need to be updated -- only if new distance is shorter */
						if(d1 < it3->second) {
							if(q.erase(node{it3->second,n1,0}) != 1) {
								fprintf(stderr,"Error: node %lu not found in the queue (distance: %f)!\n",n1,it3->second);
								return false;
							}
							it3->second = d1;
							q.insert(node{d1,n1,current});
						}
					}
				}
			} while(q.size());
			
			/* output distances found in order */
			for(size_t j = 0; j < nnodes; j++) {
				double d = 0.0;
				auto it = node_distances.find(node_ids_reverse[j]);
				if(it != node_distances.end()) d = it->second;
				node_distances2[j] = d;
				/* note: it is guaranteed that node IDs can fit in 32-bit integer above */
				if(use_index) node_sorted_ix[j] = (uint32_t)j;
			}
			node_distances.clear();
			
			/* output distances */
			for(size_t j = 0; j < nnodes; j++) {
				double d1 = node_distances2[j];
				if(std::numeric_limits<T>::is_integer) d1 = round(d1);
				if(d1 > (double)std::numeric_limits<T>::max()) {
					fprintf(stderr,"Overflow for distance: %f!\n", d1);
					return false;
				}
				T x = (T)d1;
				d.distance_file[i*nnodes + j] = x;
			}
			
			if(use_index) {
				/* sort distances */
				std::sort(node_sorted_ix.begin(), node_sorted_ix.end(),
					[this, i](uint32_t x, uint32_t y) {
						double d1 = node_distances2[x];
						double d2 = node_distances2[y];
						if(d1 == d2) {
							/* ensure that "travel" between the same locations is always the
							 * shortest even if there are zero distances to other locations */
							if(i == x && i != y) return true;
							if(i != x && i == y) return false;
						}
						return d1 < d2;
					});
				for(size_t j = 0; j < nnodes; j++) d.index_file[i*nnodes + j] = node_sorted_ix[j];
			}
			return true;
		}
		
	public:
		distance_worker(distance_worker_data& data, size_t thread_id_, size_t threads) :
				d(data), thread_id(thread_id_), nthreads(threads),
				nnodes(d.node_ids_reverse.size()), error(false) {
			node_distances2.resize(nnodes);
			if(d.index_file.is_mapped()) node_sorted_ix.resize(nnodes);
		}
		
		void run(size_t debug_output = 100) {
			while(true) {
				size_t i = d.nodes_completed++;
				if(i >= nnodes) break;
				bool res = calculate_one(i);
				if(!res) { error = true; return; }
				if(debug_output) {
					i++;
					if(i % debug_output == 0 || i == nnodes)
						fprintf(stderr,"\r%lu / %lu nodes processed", i, nnodes);
				}
			}
			
			/*
			for(size_t i = thread_id; i < nnodes; i += nthreads) {
				bool res = calculate_one(i);
				if(!res) { error = true; return; }
				if(debug_output) {
					uint64_t done = ++d.nodes_completed;
					if(done % debug_output == 0 || done == nnodes)
						fprintf(stderr,"\r%lu / %lu nodes processed", done, nnodes);
				}
					
			}*/
		}
		
		bool had_error() const { return error; }
};


template<class T> bool run_distances(const std::unordered_map<uint64_t,std::unordered_map<uint64_t,double > >& n,
			const std::vector<uint64_t>& node_ids_reverse, size_t nthreads, const char* output_fn,
			const char* index_fn = 0, size_t debug_output = 100) {
	FileMappingT<T> distance_file;
	FileMappingT<uint32_t> index_file;
	
	if(!distance_file.open_file(output_fn, FileMapping::Mode::ReadWrite, true)) {
		fprintf(stderr,"Error opening output file %s!\n",output_fn);
		return false;
	}
	size_t nnodes = node_ids_reverse.size();
	size_t dist_size = nnodes*nnodes*sizeof(T);
	if(!distance_file.change_file_size(dist_size)) {
		fprintf(stderr,"Error changing output file size!\n");
		return false;
	}
	if(!distance_file.map_file(FileMapping::Mode::ReadWrite, true)) {
		fprintf(stderr,"Error mapping output file!\n");
		return false;
	}
	
	if(index_fn) {
		size_t index_size = nnodes*nnodes*sizeof(uint32_t);
		if(!index_file.open_file(index_fn, FileMapping::Mode::ReadWrite, true)) {
			fprintf(stderr,"Error opening output file %s!\n",index_fn);
			return false;
		}
		if(!index_file.change_file_size(index_size)) {
			fprintf(stderr,"Error changing index output file size!\n");
			return false;
		}
		if(!index_file.map_file(FileMapping::Mode::ReadWrite, true)) {
			fprintf(stderr,"Error mapping index output file!\n");
			return false;
		}
	}
	
	typename distance_worker<T>::distance_worker_data data(distance_file, index_file,
			n, node_ids_reverse);
	if(nthreads > 1) {
		std::vector<std::thread> threads;
		std::vector<distance_worker<T> > workers;
		for(size_t i = 0; i < nthreads; i++) workers.emplace_back(data, i, nthreads);
		for(size_t i = 0; i < nthreads; i++) threads.emplace_back(&distance_worker<T>::run, &workers[i], debug_output);
		for(size_t i = 0; i < nthreads; i++) threads[i].join();
		for(size_t i = 0; i < nthreads; i++) if(workers[i].had_error())
			return false;
	}
	else {
		distance_worker<T> worker(data, 0, 1);
		worker.run(debug_output);
		if(worker.had_error()) return false;
	}
	return true;
}

enum class output_types {
	uint16,
	uint32,
	uint64,
	float_t,
	double_t,
	unknown
};

output_types get_type(const char* s) {
	if(s) {
		if(!strcasecmp(s,"uint16")) return output_types::uint16;
		if(!strcasecmp(s,"uint32")) return output_types::uint32;
		if(!strcasecmp(s,"uint64")) return output_types::uint64;
		if(!strcasecmp(s,"float")) return output_types::float_t;
		if(!strcasecmp(s,"double")) return output_types::double_t;
	}
	return output_types::unknown;
}

int main(int argc, char **argv)
{
	char* network_fn = 0; /* input: network file (with distances for each edge; symmetrized when reading) */
	char* output_fn = 0;
	double output_factor = 1.0; /* divide output distances by this factor (to fit in the resulting data type) */
	output_types output_type = output_types::double_t;
	char* output_sorted_index = 0; /* output file for sorted index -- sorting is done before rounding */
	size_t nthreads = 1;
	size_t debug_output = 100;
	
	for(int i=1;i<argc;i++) {
		if(argv[i][0] == '-') switch(argv[i][1]) {
			case 'n':
				network_fn = argv[i+1];
				i++;
				break;
			case 'o':
				output_fn = argv[i+1];
				i++;
				break;
			case 's':
				output_sorted_index = argv[i+1];
				i++;
				break;
			case 't':
				output_type = get_type(argv[i+1]);
				if(output_type == output_types::unknown)
					fprintf(stderr,"Unsupported argument for output type: %s!\n",argv[i+1]);
				i++;
				break;
			case 'T':
				nthreads = atoi(argv[i+1]);
				i++;
				break;
			case 'D':
				debug_output = atoi(argv[i+1]);
				i++;
				break;
			default:
				fprintf(stderr,"Unknown parameter: %s!\n",argv[i]);
				break;
		}
		else fprintf(stderr,"Unknown parameter: %s!\n",argv[i]);
	}
	
	if(output_type == output_types::unknown) {
		return 1;
	}
	
	/* read the network */
	 // graph is simply an associative container of edges with distances and counts (of trips using the edge)
	std::unordered_map<uint64_t,std::unordered_map<uint64_t,double > > n;
	/* we need an extra mapping to node IDs to ID in the matrix */
	std::vector<uint64_t> node_ids_reverse;
	size_t nnodes = 0;
	const size_t max_node_id = std::numeric_limits<uint32_t>::max();
	{
		std::unordered_map<uint64_t,size_t> node_ids;
		read_table2 rt(network_fn,stdin);
		while(rt.read_line()) {
			uint64_t n1,n2;
			double d;
			if(!rt.read(n1,n2,d)) break;
			n[n1][n2] = d;
			n[n2][n1] = d;
			
			if(node_ids.insert(std::make_pair(n1,nnodes)).second) {
				if(nnodes == max_node_id) throw std::runtime_error("Maximum number of nodes exceeded!\n");
				nnodes++;
			}
			if(node_ids.insert(std::make_pair(n2,nnodes)).second) {
				if(nnodes == max_node_id) throw std::runtime_error("Maximum number of nodes exceeded!\n");
				nnodes++;
			}
		}
		if(rt.get_last_error() != T_EOF) {
			fprintf(stderr,"Error reading network:\n");
			rt.write_error(stderr);
			return 1;
		}
		
		node_ids_reverse.resize(nnodes);
		for(const auto& x : node_ids) node_ids_reverse[x.second] = x.first;
	}
	
	bool ret = false;
	switch(output_type) {
		case output_types::uint16:
			ret = run_distances<uint16_t>(n, node_ids_reverse, nthreads,
				output_fn, output_sorted_index, debug_output);
			break;
		case output_types::uint32:
			ret = run_distances<uint32_t>(n, node_ids_reverse, nthreads,
				output_fn, output_sorted_index, debug_output);
			break;
		case output_types::uint64:
			ret = run_distances<uint64_t>(n, node_ids_reverse, nthreads,
				output_fn, output_sorted_index, debug_output);
			break;
		case output_types::float_t:
			ret = run_distances<float>(n, node_ids_reverse, nthreads,
				output_fn, output_sorted_index, debug_output);
			break;
		case output_types::double_t:
			ret = run_distances<double>(n, node_ids_reverse, nthreads,
				output_fn, output_sorted_index, debug_output);
			break;
	}
	if(!ret) {
		fprintf(stderr,"Error calculating distances!\n");
		return 1;
	}
	
	/* write out the mapping for node IDs */
	FILE* fout = stdout;
	for(size_t i = 0; i < nnodes; i++) fprintf(fout,"%lu\n",node_ids_reverse[i]);

	return 0;
}

