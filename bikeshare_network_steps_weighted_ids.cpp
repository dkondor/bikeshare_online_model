/*
 * bikeshare_network_steps_weighted_ids.cpp
 * 
 * on-line simulation of autonomous scooter fleet operation
 * 
 * maximum matching is performed step-by-step -- new step is calculated every t_1 minutes
 * 
 * in each step, events (trip start and ends) are considered and a network is created
 * 
 * additionally: keep track of trip IDs, it is possible to keep track of trip chains
 * 
 * Copyright 2019 Daniel Kondor <kondor.dani@gmail.com>
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


#include "read_table.h"
#include "distance_calculator.h"
#include "matching.h"

#include <stdio.h>
#include <time.h>
#include <vector>
#include <utility>
#include <set>
#include <unordered_set>
#include <unordered_map>


struct trip_event {
	uint64_t n; /* node id */
	double node_dist; /* extra distances from start or end nodes */
	unsigned int ts; /* timestamp */
	unsigned int id; /* trip id */
	/* sort trips by timestamp by default */
	bool operator < (const trip_event& e) const {
		if(ts < e.ts) return true;
		if(ts > e.ts) return false;
		return id < e.id;
	}
};




struct bikeshare_calculate {
	protected:
		/* available vehicles at nodes */
		std::unordered_map<uint64_t,std::unordered_set<unsigned int> > nodes_vehicles;
		std::unordered_map<unsigned int,uint64_t> vehicles_nodes;
		
		/* trip starts, sorted by time */
		std::vector<trip_event> start_events;
		/* trip ends, sorted by time -- use set, since trip ends can change (can happen later, if trip start is delayed) */
		std::set<trip_event> end_events;
		/* delays for trip ends -- checked when processing trip end event */
		std::unordered_map<unsigned int,unsigned int> trip_end_delay;
		/* start events (index) sorted by node ID and time */
		std::vector<size_t> nix;
		/* index (by node ID) into the previous */
		std::unordered_map<uint64_t,std::pair<size_t,size_t> > nix2;
		
		/* store trip start and end events that have been matched */
		std::unordered_set<unsigned int> matched_start;
		std::unordered_set<unsigned int> matched_end;
		
		/* current (simulation) time */
		unsigned int t;
		/* current index in events */
		size_t ix1; /* for trip starts */
		node_distances nd;
		spindex_adapter<uint16_t> spindex;
		
		/* total parking -- also used to get IDs for new parking spaces */
		unsigned int bikes_total;
		unsigned int bikes_counter; /* temp counter to use as IDs in nodes_vehicles */
		unsigned int trips_served; /* number of trips succesfully served */
		
		unsigned int time0; /* timestamp for start of calculations */
		/* speed of vehicles */
		double speed;
		/* distance passengers may be willing to walk before the start of a trip */
		unsigned int max_walk_dist;
		/* walking speed for passengers (in m/s) */
		double walk_speed;
		/* "combined distance" -- this includes the passenger walking and the vehicle moving together */
		double walk_combined_dist;
		double walk_combined_time;
		
		void calculate_walking_dist() {
			if(walk_speed > 0.0) {
				double tmp = max_walk_dist;
				walk_combined_time = tmp / walk_speed; /* time for walking */
				walk_combined_dist = tmp + speed * walk_combined_time;
			}
		}
		
	public:
		explicit bikeshare_calculate() : t(0), ix1(0), bikes_total(0U), time0(0U), bikes_counter(0U),
			trips_served(0U), speed(1.0/3.6), max_walk_dist(0U), walk_speed(1.0), max_vehicles(0U) { }
		bool read_network(read_table2&& rt) { return nd.read_network(std::forward<read_table2>(rt)); }
		bool read_improved(read_table2&& rt, double improved_edge_weight) { return nd.read_improved(std::forward<read_table2>(rt),improved_edge_weight); }
		bool open_spindex(const char* dists_fn, const char* sorted_fn, const char* ids_fn) {
			bool ret = spindex.open(dists_fn);
			if(!ret) return false;
			ret = spindex.open_sorted_file(sorted_fn);
			if(!ret) { spindex.close(); return false; }
			ret = spindex.read_mapping(ids_fn);
			if(!ret) { spindex.close_sorted_file(); spindex.close(); return false; }
			return true;
		}
		
		unsigned int get_ts() const { return t; }
		unsigned int get_num_vehicles() const { return bikes_total; }
		unsigned int get_trips_served() const { return trips_served; }
		unsigned int max_vehicles;
		
		/* set the maximum distance for walking along with the walking speed */
		void set_walking(unsigned int max_dist, double walk_speed) {
			max_walk_dist = max_dist;
			this->walk_speed = walk_speed;
			calculate_walking_dist();
		}
		
		/* set speed of PMD vehicles */
		void set_speed(double speed_) {
			speed = speed_;
			calculate_walking_dist();
		}
		
		bool read_events(read_table2&& rt) {
			start_events.clear();
			end_events.clear();
			matched_start.clear();
			t = 0;
			ix1 = 0;
			//~ ix2 = 0;
			//~ ix3 = 0;
			bikes_total = 0;
			//~ dist_total = 0;
			//~ trips_happening = 0;
			bikes_counter = 0;
			trips_served = 0;
			while(rt.read_line()) {
				unsigned int id,ts1,ts2;
				uint64_t node1,node2,bike_id;
				double dist_start,dist_end;
				if(!rt.read(id,bike_id,ts1,ts2,node1,dist_start,node2,dist_end)) break;
				
				start_events.push_back(trip_event{node1,dist_start,ts1,id});
				end_events.insert(trip_event{node2,dist_end,ts2,id});
			}
			if(rt.get_last_error() != T_EOF || start_events.size() == 0) {
				fprintf(stderr,"bikeshare_calculate::read_events(): ");
				rt.write_error(stderr);
				start_events.clear();
				end_events.clear();
				return false;
			}
			std::sort(start_events.begin(),start_events.end());
			//~ std::sort(end_events.begin(),end_events.end());
			
			/* create index for start events to be used for searching */
			nix.reserve(start_events.size());
			for(size_t i = 0; i < start_events.size(); i++) nix.push_back(i);
			std::sort(nix.begin(),nix.end(),[this](size_t x, size_t y) {
				if(start_events[x].n < start_events[y].n) return true;
				if(start_events[x].n > start_events[y].n) return false;
				return start_events[x] < start_events[y];
			});
			/* create index by node ID for this */
			{
				size_t i = 0;
				uint64_t n = start_events[nix[i]].n;
				nix2[n] = std::make_pair(i,0UL);
				for(i=1;i<start_events.size();i++) {
					if(start_events[nix[i]].n != n) {
						nix2[n].second = i;
						n = start_events[nix[i]].n;
						nix2[n] = std::make_pair(i,0);
					}
				}
				nix2[n].second = i; // trips.size();
			}
			
			t = start_events[0].ts;
			unsigned int t2 = end_events.cbegin()->ts;
			if(t2 < t) t = t2;
			
			return true;
		}
	
	protected:
		
		using edge = matching_helper::edge;
		/*
		struct edge {
			unsigned int first;
			unsigned int second;
			unsigned int dist;
			edge() : first(0),second(0),dist(0) { } // default constructor sets elements to avoid undefined behavior
			edge(unsigned int first_, unsigned int second_, unsigned int dist_):first(first_),second(second_),dist(dist_) { }
		}; */
		
		/* general sentinel for all dynamic iterators to signal end of iteration */
		struct iterator_sentinel { };
		
		/* iterator for creating shareability among trips */
		template<class spindex_type>
		struct trips_graph_iterator {
			const bikeshare_calculate& p;
			std::set<trip_event>::const_iterator ix; /* current index in end_events */
			unsigned int tmax; /* maximum timestamp to consider for trip end events */
			unsigned int tmax_start; /* maximum timestamp to consider for trip start events */
			bool is_end() const { return ix == p.end_events.cend() || ix->ts >= tmax; }
			bool operator == (const iterator_sentinel& dummy) const { return is_end(); }
			bool operator != (const iterator_sentinel& dummy) const { return !is_end(); }
			const std::unordered_set<unsigned int>& exclude_matched; /* matched start events */
			
			typename spindex_type::distance_iterator it;
			typename spindex_type::sentinel it_end;
			const spindex_type& spindex;
			
			size_t j; /* index in nix */
			size_t start_ix; /* range for j */
			size_t end_ix;
			edge res; /* current matching trip */
			
			double speed;
			unsigned int max_connection_time;
			unsigned int connection_time;
			unsigned int max_wait_time;
			unsigned int t2;
			
			trips_graph_iterator(const bikeshare_calculate& p_, const spindex_type& spindex_,
				unsigned int tmax_, unsigned int tmax_start_,
				const std::unordered_set<unsigned int>& exclude_matched_, double speed_,
				unsigned int max_connection_time_, unsigned int max_wait_time_)
					:p(p_),tmax(tmax_),tmax_start(tmax_start_),exclude_matched(exclude_matched_),
					spindex(spindex_),speed(speed_),max_connection_time(max_connection_time_),
					max_wait_time(max_wait_time_) {
				ix = p.end_events.cbegin();
				while ( ! (is_end() || new_ix()) ) ++ix;
			}
			void operator++() { advance(); }
			void operator++(int) { advance(); }
			const edge& operator *() const { return res; }
			const edge* operator ->() const { return &res; }
			
			bool new_ix() {
				/* if the start event corresponding to this trip wasn't processed yet, we skip this
				 * (since the end event might be delayed as well) */
				if(!exclude_matched.count(ix->id)) return false;
				res.first = ix->id;
				it = spindex.begin(ix->n);
				it_end = spindex.end(ix->n);
				return new_i();
			}
			
			bool new_i() {
				for(;it!=it_end;++it) {
					double connection_dist = round(it->d) + ix->node_dist;
					if(p.max_walk_dist > 0 && p.walk_speed > 0.0) {
						if(connection_dist >= p.walk_combined_dist)
							connection_time = (unsigned int)floor(p.walk_combined_time + 
								(connection_dist - p.walk_combined_dist) / speed);
						else connection_time = (unsigned int)floor(connection_dist / (speed + p.walk_speed));
					}
					else connection_time = (unsigned int)floor(connection_dist / speed);
					if(max_connection_time && connection_time > max_connection_time) return false;
					uint64_t n = it->node_id;
					
					{
						auto it = p.nix2.find(n);
						if(it == p.nix2.end()) continue;
						start_ix = it->second.first;
						end_ix = it->second.second;
					}
					
					t2 = ix->ts + connection_time;
					unsigned int t1 = 0;
					if(max_wait_time < t2) t1 = t2 - max_wait_time;
					
					auto is = std::lower_bound(p.nix.cbegin() + start_ix, p.nix.cbegin() + end_ix, t1,
						[this](unsigned int x, unsigned int y) { return p.start_events[x].ts < y; });
					j = is - p.nix.cbegin();
					
					if(j >= end_ix || p.start_events[p.nix[j]].ts > tmax_start) continue;
					if(!new_j()) if(!advance_j()) continue;
					
					res.second = p.nix[j];
					if(t2 > p.start_events[p.nix[j]].ts) res.dist = t2 - p.start_events[p.nix[j]].ts;
					else res.dist = 0;
					return true;
				}
				return false;
			}
			
			bool new_j() {
				if(exclude_matched.count(p.start_events[ p.nix[j] ].id)) return false;
				double connection_dist = p.start_events[p.nix[j]].node_dist + round(it->d) + ix->node_dist;
				if(p.max_walk_dist > 0 && p.walk_speed > 0.0) {
					if(connection_dist >= p.walk_combined_dist)
						connection_time = (unsigned int)floor(p.walk_combined_time + 
							(connection_dist - p.walk_combined_dist) / speed);
					else connection_time = (unsigned int)floor(connection_dist / (speed + p.walk_speed));
				}
				else t2 = ix->ts + (unsigned int)ceil( connection_dist / speed);
				//~ t2 = ix->ts + (unsigned int)ceil( (p.start_events[p.nix[j]].node_dist + round(it->d) + ix->node_dist) / speed);
				if(p.start_events[p.nix[j]].ts + max_wait_time < t2) return false;
				return true;
			}
			
			bool advance_j() {
				bool ret = false;
				while(true) {
					j++;
					if(j >= end_ix || p.start_events[p.nix[j]].ts > tmax_start) return false;
					if(new_j()) return true;
				}
			}
			
			void advance() {
				if(is_end()) return;
				if(!advance_j()) {
					++it;
					if(!new_i()) do ++ix; while( ! (is_end() || new_ix()) );
				}
				else {
					res.second = p.nix[j];
					if(t2 > p.start_events[p.nix[j]].ts) res.dist = t2 - p.start_events[p.nix[j]].ts;
					else res.dist = 0;
				}
			}
			
		};
		
		template<class spindex_type>
		struct trip_start_iterator {
			const bikeshare_calculate& p;
			const spindex_type& spindex;
			typename spindex_type::distance_iterator it;
			typename spindex_type::sentinel it_end;
			std::unordered_set<unsigned int>::const_iterator it2;
			std::unordered_set<unsigned int>::const_iterator it2_end;
			
			const std::unordered_set<unsigned int>& exclude_matched;
			size_t ix;
			
			edge res; /* current matching trip */
			
			double speed;
			
			
			unsigned int tmax; /* maximum timestamp to consider for trip start events */
			unsigned int ts; /* current (decision) timestamp -- it is assumed that vehicles can start moving at this time */
			unsigned int t2; /* time vehicles from the current node can reach the trip start */
			unsigned int max_wait_time;
			bool is_end() const { return ix >= p.start_events.size() || p.start_events[ix].ts >= tmax; }
			bool operator == (const iterator_sentinel& dummy) const { return is_end(); }
			bool operator != (const iterator_sentinel& dummy) const { return !is_end(); }
			
			void operator++() { advance(); }
			void operator++(int) { advance(); }
			const edge& operator *() const { return res; }
			const edge* operator ->() const { return &res; }
			
			trip_start_iterator(const bikeshare_calculate& p_, const spindex_type& spindex_, size_t ix_, unsigned int tmax_, unsigned int ts_,
				unsigned int max_wait_time_, double speed_, const std::unordered_set<unsigned int>& exclude_matched_)
					:p(p_),spindex(spindex_),exclude_matched(exclude_matched_),ix(ix_),speed(speed_),tmax(tmax_),ts(ts_),max_wait_time(max_wait_time_) {
				while ( ! (is_end() || new_ix()) ) ix++;
			}
			
			bool new_ix() {
				res.first = ix;
				if(exclude_matched.count(p.start_events[ix].id)) return false;
				
				it = spindex.begin(p.start_events[ix].n);
				it_end = spindex.end(p.start_events[ix].n);
				return new_i();
			}
			
			bool new_i() {
				for(;it!=it_end;++it) {
					unsigned int connection_time;
					
					double connection_dist = round(it->d) + p.start_events[ix].node_dist;
					if(p.max_walk_dist > 0 && p.walk_speed > 0.0) {
						if(connection_dist >= p.walk_combined_dist)
							connection_time = (unsigned int)floor(p.walk_combined_time + 
								(connection_dist - p.walk_combined_dist) / speed);
						else connection_time = (unsigned int)floor(connection_dist / (speed + p.walk_speed));
					}
					else connection_time = (unsigned int)floor(connection_dist / speed);
					t2 = ts + connection_time;
					if(t2 > p.start_events[ix].ts + max_wait_time) return false;
					auto it3 = p.nodes_vehicles.find(it->node_id);
					if(it3 == p.nodes_vehicles.cend()) continue;
					it2 = it3->second.begin();
					it2_end = it3->second.end();
					if(it2 == it2_end) continue;
					res.second = *it2;
					if(t2 > p.start_events[ix].ts) res.dist = t2 - p.start_events[ix].ts;
					else res.dist = 0;
					return true;
				}
				return false;
			}
			
			
			void advance() {
				if(is_end()) return;
				++it2;
				if(it2 == it2_end) {
					++it;
					if(!new_i()) do ix++; while( ! (is_end() || new_ix()) );
				}
				else res.second = *it2;
			}
		};
		
		
		
		/* process end_events between t and t+ts, assume vehicles park at the closest node */
		void process_end_events(unsigned int ts, FILE* fdetail, bool only_delays = false) {
			for(auto it = end_events.begin(); it != end_events.end() && it->ts < t + ts; ) {
				unsigned int trip_id = it->id;
				if(matched_end.count(trip_id)) it = end_events.erase(it);
				else {
					uint64_t n = it->n;
					unsigned int t1 = it->ts;
					unsigned int t2 = t1 + (unsigned int)ceil(it->node_dist / speed);
					
					auto it2 = trip_end_delay.find(trip_id);
					if(it2 != trip_end_delay.end()) {
						unsigned int delay = it2->second;
						trip_end_delay.erase(it2);
						t2 += delay;
						if(only_delays || t2 >= t + ts) { /* erase and reinsert in this case */
							trip_event t1 = *it;
							it = end_events.erase(it);
							t1.ts += delay;
							end_events.insert(t1);
							continue;
						}
					}
					else if(only_delays || t2 >= t + ts) {
						++it; /* do not adjust yet if the vehicle cannot reach the "node" exactly before t + ts */
						continue;
					}
					
					/* here t2 < t + ts, we need to "park" the vehicle at node n */
					it = end_events.erase(it);
					/* add vehicle to node n */
					unsigned int vehicle_id = bikes_counter;
					bikes_counter++;
					nodes_vehicles[n].insert(vehicle_id);
					vehicles_nodes.insert(std::make_pair(vehicle_id,n));
					/* write output as parking event */
					if(fdetail) fprintf(fdetail,"3\t%u\t%lu\t0\n",trip_id,n);
				}
			}
		}
	
	public:
	
		/* advance the simulation by ts from current time (t)
		 * treserve parameter gives if trips are reserve in advance:
		 * if it's true, we assume that the current time is t, and we know all trip reservations in [t,t+ts]
		 * otherwise, we assume that the current time is t+ts and we pooled reservations since t */
		void advance(unsigned int ts, bool treserve, unsigned int max_wait_time, FILE* fdetail, FILE* fout = 0, bool limit_new = false) {
			/* 0. update trip end times and remove matched end events if needed */
			/* if the "current time" is t + ts, we need to process end_events that happen before this time
			 * otherwise, this processing happens at the end of this function
			 * in both cases, in the next step, end_events only contains "valid" events */
			if(!treserve) process_end_events(ts,fdetail);
			else process_end_events(ts,fdetail,true); /* only add delays to events in the new time interval */
			
			/* 1. create shareability network among the trips 
					trips_graph_iterator(const bikeshare_calculate& p_, unsigned int tmax_, unsigned int tmax_start_,
				const std::unordered_set<unsigned int>& exclude_matched_, double speed_,
				unsigned int max_connection_time_, unsigned int max_wait_time_) */
			{
				std::vector<edge> matching;
				unsigned int tmax_end = t;
				if(treserve) tmax_end = t + ts;
				if(fout) fprintf(fout,"%u, calculating matching among trips, ",(unsigned int)time(0)-time0);
				
				if(spindex.is_sorted()) {
					trips_graph_iterator<spindex_adapter<uint16_t> > it(*this, spindex,
						tmax_end, t+ts, matched_start, speed, 0, max_wait_time);
					matching_helper::calculate_matching(it, iterator_sentinel(), 
						matching, max_wait_time, fout);
				}
				else {
					trips_graph_iterator<node_distances> it(*this, nd, tmax_end, t+ts,
						matched_start, speed, 0, max_wait_time);
					matching_helper::calculate_matching(it, iterator_sentinel(),
						matching, max_wait_time, fout);
				}
				/* process these matches */
				for(auto x : matching) {
					/* x.first is the trip ID of the end trip that has been matched,
					 * x.second is the index into the start events array that has been matched */
					matched_end.insert(x.first);
					unsigned int start_id = start_events[x.second].id;
					matched_start.insert(start_id);
					trips_served++;
					
					unsigned int delay = x.dist;
					if(delay > 0) trip_end_delay.insert(std::make_pair(start_id,delay));
					
					/* write out matching */
					if(fdetail) fprintf(fdetail,"1\t%u\t%u\t%u\n",x.first,start_id,delay);
				}
			}
			
			{	
				/* 2. match trip starts to vehicles available; process start events in [t, t + ts] 
				trip_start_iterator(const bikeshare_calculate& p_, size_t ix_, unsigned int tmax_, unsigned int ts_,
				unsigned int max_wait_time_, double speed_, const std::unordered_set<unsigned int>& exclude_matched_)
					:p(p_),exclude_matched(exclude_matched_),ix(ix_),speed(speed_),tmax(tmax_),ts(ts_),max_wait_time(max_wait_time_) */
				std::vector<edge> start_matching;
				if(fout) fprintf(fout,"%u, calculating matching for trip starts, ",(unsigned int)time(0)-time0);
				unsigned int t1 = t;
				if(!treserve) t1 = t + ts;
				if(spindex.is_sorted()) {
					trip_start_iterator<spindex_adapter<uint16_t> > it(*this, spindex,
						ix1, t + ts, t1, max_wait_time, speed, matched_start);
					matching_helper::calculate_matching(it, iterator_sentinel(),
						start_matching, max_wait_time, fout);
				}
				else {
					trip_start_iterator<node_distances> it(*this, nd,
						ix1, t + ts, t1, max_wait_time, speed, matched_start);
					matching_helper::calculate_matching(it, iterator_sentinel(),
						start_matching, max_wait_time, fout);
				}
				for(auto x : start_matching) {
					
					unsigned int start_ix = x.first;
					unsigned int vehicle_id = x.second;
					unsigned int delay = x.dist;
					unsigned int start_id = start_events[start_ix].id;
					
					matched_start.insert(start_id);
					auto it = vehicles_nodes.find(vehicle_id);
					if(it == vehicles_nodes.end()) throw std::runtime_error("Error processing trip start matches!\n");
					uint64_t n = it->second;
					vehicles_nodes.erase(it);
					nodes_vehicles[n].erase(vehicle_id);
					trips_served++;
					
					if(delay > 0) trip_end_delay.insert(std::make_pair(start_id,delay));
					
					/* write out matching */
					if(fdetail) fprintf(fdetail,"2\t%u\t%lu\t%u\n",start_id,n,delay);
				}
			}
			
			/* 3. process remaining start events by creating new vehicles */
			for(;ix1 < start_events.size() && start_events[ix1].ts < t + ts; ix1++) if(!matched_start.count(start_events[ix1].id)) {
				uint64_t n = start_events[ix1].n;
				unsigned int start_id = start_events[ix1].id;
				unsigned int t1 = start_events[ix1].ts;
				unsigned int delay = (unsigned int)ceil(start_events[ix1].node_dist / speed);
				if(treserve) {
					if(delay < ts) delay = 0;
					else delay -= ts;
				}
				if( (limit_new && delay > max_wait_time) || (max_vehicles && bikes_total >= max_vehicles) ) {
					matched_end.insert(start_id); /* skip trip if cannot reach start within max wait time */
					continue;
				}
				matched_start.insert(start_id); /* signal that this start event has been processed already */
				if(delay) trip_end_delay.insert(std::make_pair(start_id,delay));
				if(fdetail) fprintf(fdetail,"2\t%u\t%lu\t%u\n",start_id,n,delay);
				bikes_total++;
				trips_served++;
			}
			
			/* if advance reservations are used ("current time" is t), we need to process remaining end_events betweet t and t + ts */
			if(treserve) process_end_events(ts,fdetail);
			
			t += ts;
		}
	
		bool is_end() const { return ix1 >= start_events.size() && end_events.empty(); }
};


int main(int argc, char **argv)
{
	unsigned int ts = 300;
	unsigned int max_wait_time = 300;
	char* trips = 0;
	char* network_fn = 0;
	double speed = 5000.0 / 3600.0;
	char* detail_out = 0;
	char* improved_edges = 0; /* optionally: list of edges which have been improved (allow faster travel) */
	double improved_edge_weight = 1.5; /* extra preference toward improved edges */
	bool treserve = false;
	unsigned int max_vehicles = 0;
	bool limit_new = false;
	char* spindex_fn = 0; /* precalculated spatial index (for travel distances among nodes) */
	char* dists_fn = 0; /* precalculated distances between all nodes */
	char* ids_mapping_fn = 0; /* mapping of IDs to matrix locations for the previous */
	unsigned int max_walk_dist = 0; /* in meters */
	double walking_speed = 1.0; /* in m/s */
	
	for(int i=1;i<argc;i++) {
		if(argv[i][0] != '-') fprintf(stderr,"Unknown command line argument: %s!\n",argv[i]);
		else switch(argv[i][1]) {
			case 'o':
				detail_out = argv[i+1];
				i++;
				break;
			case 't':
				trips = argv[i+1];
				i++;
				break;
			case 'n':
				network_fn = argv[i+1];
				i++;
				break;
			case 'T':
				ts = atoi(argv[i+1]);
				i++;
				break;
			case 'w':
				max_wait_time = atoi(argv[i+1]);
				i++;
				break;
			case 's':
				speed = atof(argv[i+1]) / 3.6; // should be given in km/h
				i++;
				break;
			case 'I':
				improved_edge_weight = atof(argv[i+1]);
				i++;
				break;
			case 'i':
				improved_edges = argv[i+1];
				i++;
				break;
			case 'r':
				treserve = true;
				break;
			case 'l':
				limit_new = true;
				break;
			case 'm':
				max_vehicles = atoi(argv[i+1]);
				i++;
				break;
			case 'd':
				if(i+3 >= argc) {
					fprintf(stderr,"Invalid arguments for -d (need 3 filenames)!\n");
					break;
				}
				dists_fn = argv[i+1];
				spindex_fn = argv[i+2];
				ids_mapping_fn = argv[i+3];
				i+=3;
				break;
			case 'W':
				if(argv[i][2] == 's' || argv[i][2] == 'S') {
					walking_speed = atof(argv[i+1]) / 3.6;
					if(walking_speed <= 0.0) fprintf(stderr,"Invalid argument for walking speed: %s %s!\n",argv[i],argv[i+1]);
					i++;
					break;
				}
				if(argv[i][2] == 'd' || argv[i][2] == 'D') {
					max_walk_dist = atoi(argv[i+1]);
					i++;
					break;
				}
			default:
				fprintf(stderr,"Unknown command line argument: %s!\n",argv[i]);
				break;
		}
	}
	
	if( ! (network_fn || spindex_fn) ) {
		fprintf(stderr,"Missing parameters!\n");
		return 1;
	}
	
	bikeshare_calculate p;
	if(network_fn) {
		if(!p.read_network(read_table2(network_fn))) {
			fprintf(stderr,"Error reading path network!\n");
			return 1;
		}
		if(improved_edges) if(!p.read_improved(read_table2(improved_edges),improved_edge_weight)) {
			fprintf(stderr,"Error reading improved edge list!\n");
			return 1;
		}
	}
	else if(!p.open_spindex(dists_fn, spindex_fn, ids_mapping_fn)) {
		fprintf(stderr,"Error opening distances or index file!\n");
		return 1;
	}
	
	if(!p.read_events(read_table2(trips,stdin))) {
		fprintf(stderr,"Error reading trips!\n");
		return 1;
	}
	p.max_vehicles = max_vehicles;
	p.set_speed(speed);
	p.set_walking(max_walk_dist, walking_speed);
	
	FILE* fdetail = 0;
	if(detail_out) {
		fdetail = fopen(detail_out,"w");
		if(!fdetail) {
			fprintf(stderr,"Error opening output file %s!\n",detail_out);
			return 1;
		}
	}
	
	while(!p.is_end()) {
		p.advance(ts,treserve,max_wait_time,fdetail,stderr,limit_new);
		fflush(stderr);
		fprintf(stdout,"%u\t%u\t%u\n",p.get_ts(),p.get_num_vehicles(),p.get_trips_served());
		fflush(stdout);
	}
	
	return 0;
}

