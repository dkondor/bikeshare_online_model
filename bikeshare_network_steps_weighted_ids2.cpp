/*
 * bikeshare_network_steps_weighted_ids2.cpp
 * 
 * on-line simulation of autonomous scooter fleet operation; 
 * adapted from minimum parking and fleet calculation -- parking is not relevant here
 * 
 * maximum matching is performed step-by-step -- new step is calculated every t_1 minutes
 * 
 * additionally: keep track of trip IDs, it is possible to keep track of trip chains
 * 
 * 
 * different approach than previously: use pre-calculated distances among trip
 * end and start events to avoid the costly shortest paths calculations
 * 
 * 
 * Copyright 2019 Daniel Kondor <kondor.dani@gmail.com>
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
#include "matching.h"

#include <stdio.h>
#include <time.h>
#include <string.h>
#include <vector>
#include <utility>
#include <random>
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
	
		/* edge for maximum matching 
		struct edge {
			unsigned int first;
			unsigned int second;
			unsigned int dist; // note: dist is actually travel time
			edge() : first(0),second(0),dist(0) { } // default constructor sets elements to avoid undefined behavior 
			edge(unsigned int first_, unsigned int second_, unsigned int dist_):first(first_),second(second_),dist(dist_) { }
		}; */
		using edge = matching_helper::edge;
		
		/* edge for matching nodes to trip starts */
		struct node_edge {
			uint64_t node_id;
			unsigned int trip_id;
			unsigned int dist; /* note: dist is actually travel time */
			node_edge() : node_id(0),trip_id(0),dist(0) { } /* default constructor sets elements to avoid undefined behavior */
			node_edge(uint64_t node_id_, unsigned int trip_id_, unsigned int dist_):node_id(node_id_),trip_id(trip_id_),dist(dist_) { }
		};
		
	
		/* available vehicles at nodes */
		std::unordered_map<uint64_t,std::unordered_set<unsigned int> > nodes_vehicles;
		std::unordered_map<unsigned int,uint64_t> vehicles_nodes;
		
		/* travel times (from trip end events to next trip start events, pre-computed,
		 * sorted by trip ID and travel time */
		std::vector<edge> trip_connections;
		std::vector<node_edge> node_connections; /* sorted by trip ID and travel time */
		
		/* trip starts, sorted by time */
		std::vector<trip_event> start_events;
		std::unordered_map<unsigned int,size_t> start_events_ids; /* index by event IDs (needed as well for trip graph iterator) */
		
		/* trip ends, sorted by time -- use set, since trip ends can change (can happen later, if trip start is delayed) */
		std::set<trip_event> end_events;
		/* delays for trip ends -- checked when processing trip end event */
		std::unordered_map<unsigned int,unsigned int> trip_end_delay;
		
		/* store trip start and end events that have been matched */
		std::unordered_set<unsigned int> matched_start;
		std::unordered_set<unsigned int> matched_end;
		
		/* current (simulation) time */
		unsigned int t;
		/* current index in events */
		size_t ix1; /* for trip starts */
		//~ node_distances nd;
		
		/* total parking -- also used to get IDs for new parking spaces */
		unsigned int bikes_total;
		unsigned int bikes_counter; /* temp counter to use as IDs in nodes_vehicles */
		unsigned int trips_served; /* number of trips succesfully served */
		
		unsigned int time0; /* timestamp for start of calculations */
	
	public:
		explicit bikeshare_calculate() : t(0), ix1(0), bikes_total(0U), bikes_counter(0U), trips_served(0U), time0(0U), max_vehicles(0U) { }
		//~ bool read_network(read_table2&& rt) { return nd.read_network(std::forward<read_table2>(rt)); }
		//~ bool read_improved(read_table2&& rt, double improved_edge_weight) { return nd.read_improved(std::forward<read_table2>(rt),improved_edge_weight); }
		
		
		unsigned int get_ts() const { return t; }
		unsigned int get_num_vehicles() const { return bikes_total; }
		unsigned int get_trips_served() const { return trips_served; }
		unsigned int max_vehicles;
		
		bool read_events(read_table2& rt) {
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
			
			/* create index for start events to be used for searching */
			start_events_ids.clear();
			for(size_t i=0;i<start_events.size();i++) start_events_ids.insert(std::make_pair(start_events[i].id,i));
			
			t = start_events[0].ts;
			unsigned int t2 = end_events.cbegin()->ts;
			if(t2 < t) t = t2;
			
			return true;
		}
		bool read_events(read_table2&& rt) { return read_events(rt); }
		
		/* read previously calculated connection distances, convert them to connection times
		 * only keep connections where the user has to wait less than max_wait_time and the
		 * total connection travel time is <= max_wait_time + max_reserve_time */
		bool read_connections(read_table2& rt_trips, read_table2& rt_nodes, double speed, unsigned int max_wait_time,
				unsigned int max_reserve_time, bool precise_sort = false) {
			trip_connections.clear();
			node_connections.clear();
			
			/* 1. we need a dictionary of trip starts and ends by trip ID */
			//~ std::unordered_map<unsigned int, trip_event> trip_starts;
			std::unordered_map<unsigned int, trip_event> trip_ends;
			//~ for(const auto& t : start_events) trip_starts.insert(std::make_pair(t.id,t));
			for(const auto& t : end_events) trip_ends.insert(std::make_pair(t.id,t));
			
			/* 2. read possible trip connections */
			while(rt_trips.read_line()) {
				edge e;
				double dist;
				if(!rt_trips.read(e.first,e.second,dist)) break;
				unsigned int travel_time = (unsigned int)ceil(dist / speed);
				if(travel_time > max_wait_time + max_reserve_time) continue;
				e.dist = travel_time;
				const trip_event& ee = trip_ends.at(e.first);
				const trip_event& se = start_events[ start_events_ids.at(e.second) ];
				if(ee.ts + travel_time <= se.ts + max_wait_time) trip_connections.push_back(e);
			}
			if(rt_trips.get_last_error() != T_EOF) {
				fprintf(stderr,"bikeshare_calculate::read_connections: error reading trip connections:\n");
				rt_trips.write_error(stderr);
				return false;
			}
			
			/* 3. read connections to trip starts from nodes */
			while(rt_nodes.read_line()) {
				node_edge e;
				double dist;
				if(!rt_nodes.read(e.trip_id,e.node_id,dist)) break;
				unsigned int travel_time = (unsigned int)ceil(dist / speed);
				if(travel_time > max_wait_time + max_reserve_time) continue;
				e.dist = travel_time;
				node_connections.push_back(e);
			}
			if(rt_nodes.get_last_error() != T_EOF) {
				fprintf(stderr,"bikeshare_calculate::read_connections: error reading trip start -- node connections:\n");
				rt_nodes.write_error(stderr);
				return false;
			}
			
			/* 4. sort the connections */
			if(precise_sort) {
				/* sort should produce exactly the same order as in the previous implementation
				 * for this, we have to preserve the order of how nodes were found in the shortest path search
				 * 1. we do a stable sort based only on the distance */
				std::stable_sort(trip_connections.begin(),trip_connections.end(),[this](const edge& e1, const edge& e2) {
					if(e1.first < e2.first) return true;
					if(e1.first > e2.first) return false;
					return e1.dist < e2.dist;
				});
				/* 2. next, we find partitions with the same node_id and sort the trip there based on the sort order used previously
				 * we find partitions by iterating over all elements -- an O(N) operation does not add much (previous sort was alrady O(NlogN) */
				size_t i = 0, j;
				while(i < trip_connections.size()) {
					/* find partition -- problem: how to check if trip_connections is really partitioned by node_id? */
					uint64_t target_node = start_events[ start_events_ids.at(trip_connections[i].second) ].n;
					for(j=i+1; j<trip_connections.size(); j++) {
						if(trip_connections[j].first != trip_connections[i].first) break;
						if(trip_connections[j].dist != trip_connections[i].dist) break;
						if(start_events[ start_events_ids.at(trip_connections[j].second) ].n != target_node) break;
					}
					/* sort i -- j range by start_event */
					if(j > i+1) std::sort(trip_connections.begin() + i, trip_connections.begin() + j,[this](const edge& e1, const edge& e2) {
						const trip_event& t1 = start_events[ start_events_ids.at(e1.second) ];
						const trip_event& t2 = start_events[ start_events_ids.at(e2.second) ];
						return (t1 < t2);
					});
					i = j;
				}
			}
			else std::sort(trip_connections.begin(),trip_connections.end(),[](const edge& e1, const edge& e2) {
				if(e1.first < e2.first) return true;
				if(e1.first > e2.first) return false;
				return e1.dist < e2.dist;
			});
			std::stable_sort(node_connections.begin(),node_connections.end(),[](const node_edge& e1, const node_edge& e2) {
				if(e1.trip_id < e2.trip_id) return true;
				if(e1.trip_id > e2.trip_id) return false;
				return e1.dist < e2.dist;
			});
			return true;
		}
	
	
		/* place n vehicles randomly at trip start locations */
		template<class R>
		void random_init2(unsigned int nvehicles, R& rng) {
			bikes_counter = 0;
			bikes_total = 0;
			nodes_vehicles.clear();
			vehicles_nodes.clear();
			
			if(!start_events.size()) throw std::runtime_error("bikeshare_calculate::random_init(): no trips present!\n");
			std::uniform_int_distribution<size_t> dist(0UL,start_events.size()-1);
			for(unsigned int i=0;i<nvehicles;i++) {
				size_t x = dist(rng);
				uint64_t n = start_events[x].n;
				nodes_vehicles[n].insert(bikes_counter);
				vehicles_nodes[bikes_counter] = n;
				bikes_counter++;
				bikes_total++;
			}
		}
		template<class R>
		void random_init(unsigned int nvehicles, R& rng) {
			if(!start_events.size()) throw std::runtime_error("bikeshare_calculate::random_init(): no trips present!\n");
			struct ri_sent {
				size_t n;
				ri_sent():n(0UL) { }
				explicit ri_sent(size_t n_):n(n_) { }
			};
			struct rng_it {
				R& rng;
				const std::vector<trip_event>& start_events;
				size_t i;
				std::uniform_int_distribution<size_t> dist;
				uint64_t next;
				rng_it(R& rng_, const std::vector<trip_event>& s):rng(rng_),start_events(s),i(0UL),dist(0UL,start_events.size()-1),next(0UL) { advance(); }
				void advance() {
					size_t x = dist(rng);
					next = start_events[x].n;
				}
				void operator++() { advance(); ++i; }
				void operator++(int) { advance(); ++i; }
				uint64_t operator*() const { return next; }
				bool operator == (const ri_sent& s) const { return i == s.n; }
				bool operator != (const ri_sent& s) const { return i != s.n; }
			};
			dist_init(rng_it(rng,start_events),ri_sent(nvehicles));
		}
		
		
		template<class it, class sent>
		void dist_init(it it1, sent s) {
			bikes_counter = 0;
			bikes_total = 0;
			nodes_vehicles.clear();
			vehicles_nodes.clear();
			for(;it1 != s;++it1) {
				uint64_t n = *it1;
				nodes_vehicles[n].insert(bikes_counter);
				vehicles_nodes[bikes_counter] = n;
				bikes_counter++;
				bikes_total++;
			}
		}
		
		bool dist_file_init(read_table2& rt) {
			struct rt_sent { };
			struct rt_it {
				bool is_end;
				read_table2& rt;
				uint64_t next;
				explicit rt_it(read_table2& rt_):is_end(false),rt(rt_),next(0UL) { read_next(); }
				void read_next() {
					if(!rt.read_line()) is_end = true;
					else if(!rt.read(next)) is_end = true;
				}
				void operator ++() { read_next(); }
				void operator ++(int) { read_next(); }
				uint64_t operator*() const { return next; }
				bool operator == (const rt_sent& s) const { return is_end; }
				bool operator != (const rt_sent& s) const { return !is_end; }
			};
			dist_init(rt_it(rt),rt_sent());
			if(rt.get_last_error() != T_EOF) {
				fprintf(stderr,"bikeshare_calculate::dist_file_init(): error reading initial positions:\n");
				rt.write_error(stderr);
				return false;
			}
			return true;
		}
		bool dist_file_init(read_table2&& rt) { return dist_file_init(rt); }
		
	protected:
		
		/* general sentinel for all dynamic iterators to signal end of iteration */
		struct iterator_sentinel { };
		
		/* iterator for creating shareability among trips */
		struct trips_graph_iterator {
			const bikeshare_calculate& p;
			std::set<trip_event>::const_iterator ix; /* current index in end_events */
			unsigned int tmax; /* maximum timestamp to consider for trip end events */
			unsigned int tmax_start; /* maximum timestamp to consider for trip start events */
			bool is_end() const { return ix == p.end_events.cend() || ix->ts >= tmax; }
			bool operator == (const iterator_sentinel& dummy) const { return is_end(); }
			bool operator != (const iterator_sentinel& dummy) const { return !is_end(); }
			const std::unordered_set<unsigned int>& exclude_matched; /* matched start events */
			
			size_t i; /* index in trip_connections */
			edge res; /* current matching trip */
			
			double speed;
			unsigned int max_connection_time;
			unsigned int connection_time;
			unsigned int max_wait_time;
			unsigned int t2;
			
			trips_graph_iterator(const bikeshare_calculate& p_, unsigned int tmax_, unsigned int tmax_start_,
				const std::unordered_set<unsigned int>& exclude_matched_, double speed_,
				unsigned int max_connection_time_, unsigned int max_wait_time_)
					:p(p_),tmax(tmax_),tmax_start(tmax_start_),exclude_matched(exclude_matched_),speed(speed_),
					max_connection_time(max_connection_time_),max_wait_time(max_wait_time_) {
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
				
				auto it = std::lower_bound(p.trip_connections.cbegin(),p.trip_connections.cend(),ix->id,
					[](const edge& e,unsigned int id) { return e.first < id; });
				
				//~ if(it == p.trip_connections.cend() || it->first != ix->id) return false; -- handled by new_i()
				i = it - p.trip_connections.cbegin();
				return new_i();
			}
			
			bool new_i() {
				for(;i<p.trip_connections.size() && p.trip_connections[i].first == ix->id;i++) {
					if(max_connection_time && p.trip_connections[i].dist > max_connection_time) return false;
					unsigned int start_id = p.trip_connections[i].second;
					if(exclude_matched.count(start_id)) continue;
					unsigned int t1 = ix->ts;
					unsigned int t2 = t1 + p.trip_connections[i].dist;
					const trip_event& e = p.start_events[ p.start_events_ids.at(start_id) ];
					if(e.ts <= tmax_start && t2 <= e.ts + max_wait_time) {
						res.second = start_id;
						res.dist = 0;
						if(t2 > e.ts) res.dist = t2 - e.ts;
						return true;
					}
				}
				return false;
			}
			
			void advance() {
				if(is_end()) return;
				i++;
				if(!new_i()) do ++ix; while( ! (is_end() || new_ix()) ); /* note: res is set in new_i() */
			}
			
		};
		
		
		struct trip_start_iterator {
			const bikeshare_calculate& p;
			
			std::unordered_set<unsigned int>::const_iterator it2;
			std::unordered_set<unsigned int>::const_iterator it2_end;
			
			const std::unordered_set<unsigned int>& exclude_matched;
			size_t ix;
			size_t i;
			
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
			
			trip_start_iterator(const bikeshare_calculate& p_, size_t ix_, unsigned int tmax_, unsigned int ts_,
				unsigned int max_wait_time_, double speed_, const std::unordered_set<unsigned int>& exclude_matched_)
					:p(p_),exclude_matched(exclude_matched_),ix(ix_),speed(speed_),tmax(tmax_),ts(ts_),max_wait_time(max_wait_time_) {
				while ( ! (is_end() || new_ix()) ) ix++;
			}
			
			bool new_ix() {
				res.first = ix;
				if(exclude_matched.count(p.start_events[ix].id)) return false;
				
				auto it = std::lower_bound(p.node_connections.cbegin(), p.node_connections.cend(), p.start_events[ix].id,
					[](const node_edge& e, unsigned int id){ return e.trip_id < id; });
				i = it - p.node_connections.cbegin();
				return new_i();
			}
			
			bool new_i() {
				for(;i<p.node_connections.size() && p.node_connections[i].trip_id == p.start_events[ix].id; i++) {
					uint64_t node_id = p.node_connections[i].node_id;
					unsigned int connection_time = p.node_connections[i].dist;
					t2 = ts + connection_time;
					if(t2 > p.start_events[ix].ts + max_wait_time) return false;
					
					auto it3 = p.nodes_vehicles.find(node_id);
					if(it3 == p.nodes_vehicles.cend()) continue;
					it2 = it3->second.cbegin();
					it2_end = it3->second.cend();
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
					i++;
					if(!new_i()) do ix++; while( ! (is_end() || new_ix()) );
				}
				else res.second = *it2;
			}
		};
		
				
		
		/* process end_events between t and t+ts, assume vehicles park at the closest node */
		void process_end_events(unsigned int ts, double speed, FILE* fdetail) {
			for(auto it = end_events.begin(); it != end_events.end() && it->ts < t + ts; ) {
				unsigned int trip_id = it->id;
				if(matched_end.count(trip_id) || !matched_start.count(trip_id)) {
					it = end_events.erase(it);
					matched_start.insert(trip_id); /* make sure that the start of this trip cannot be matched later */
				}
				else {
					uint64_t n = it->n;
					unsigned int t1 = it->ts;
					unsigned int t2 = t1 + (unsigned int)ceil(it->node_dist / speed);
					
					auto it2 = trip_end_delay.find(trip_id);
					if(it2 != trip_end_delay.end()) {
						unsigned int delay = it2->second;
						trip_end_delay.erase(it2);
						t2 += delay;
						if(t2 >= t + ts) { /* erase and reinsert in this case */
							trip_event t1 = *it;
							it = end_events.erase(it);
							t1.ts += delay;
							end_events.insert(t1);
							continue;
						}
					}
					else if(t2 >= t + ts) {
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
		 * look ahead by tw, i.e. consider all trips in the window [t,t+tw],
		 * but only assign results for trips in [t,t+ts]
		 * treserve parameter gives if trips are reserve in advance:
		 * if it's true, we assume that the current time is t, and we know all trip reservations in [t,t+ts]
		 * otherwise, we assume that the current time is t+ts and we pooled reservations since t */
		void advance(unsigned int ts, unsigned int tw, bool treserve, unsigned int max_wait_time,
				double speed, FILE* fdetail, FILE* fout = 0, bool limit_new = false) {
			/* 0. update trip end times and remove matched end events if needed */
			/* if the "current time" is t + ts, we need to process end_events that happen before this time
			 * otherwise, this processing happens at the end of this function
			 * in both cases, in the next step, end_events only contains "valid" events */
			if(!treserve) process_end_events(ts,speed,fdetail);
			
			
			/* 1. create shareability network among the trips 
					trips_graph_iterator(const bikeshare_calculate& p_, unsigned int tmax_, unsigned int tmax_start_,
				const std::unordered_set<unsigned int>& exclude_matched_, double speed_,
				unsigned int max_connection_time_, unsigned int max_wait_time_) */
			{
				std::vector<edge> matching;
				unsigned int tmax_end = t + tw;
				//~ if(treserve) tmax_end = t + ts; -- don't understand why there is a distinction here
				// probably this should always consider all available end events that might be connected
				// to a start event
				trips_graph_iterator it(*this, tmax_end, t+tw, matched_start, speed, 0, max_wait_time);
				if(fout) fprintf(fout,"%u, calculating matching among trips, ",(unsigned int)time(0)-time0);
				matching_helper::calculate_matching(it, iterator_sentinel(),
					matching, max_wait_time, fout);
				
				/* process these matches */
				for(auto x : matching) {
					/* x.first is the trip ID of the end trip that has been matched,
					 * x.second is the index into the start events array that has been matched */
					unsigned int start_id = x.second;
					const trip_event& start1 = start_events[ start_events_ids.at(start_id) ];
					/* only proces trips before t + ts */
					if(start1.ts >= t + ts) continue;
					
					matched_end.insert(x.first);
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
				trip_start_iterator it(*this, ix1, t + tw, t1, max_wait_time, speed, matched_start);
				matching_helper::calculate_matching(it, iterator_sentinel(),
					start_matching, max_wait_time, fout);
				for(auto x : start_matching) {
					
					unsigned int start_ix = x.first;
					if(start_events[start_ix].ts >= t + ts) continue;
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
				//~ unsigned int t1 = start_events[ix1].ts;
				unsigned int delay = (unsigned int)ceil(start_events[ix1].node_dist / speed);
				if(treserve) {
					if(delay < ts) delay = 0;
					else delay -= ts;
				}
				if( (limit_new && delay > max_wait_time) || (max_vehicles && bikes_total >= max_vehicles) ) {
					//~ matched_end.insert(start_id); /* skip trip if cannot reach start within max wait time */
					continue;
				}
				matched_start.insert(start_id); /* signal that this start event has been processed already */
				if(delay) trip_end_delay.insert(std::make_pair(start_id,delay));
				if(fdetail) fprintf(fdetail,"2\t%u\t%lu\t%u\n",start_id,n,delay);
				bikes_total++;
				trips_served++;
			}
			
			/* if advance reservations are used ("current time" is t), we need to process remaining end_events betweet t and t + ts */
			if(treserve) process_end_events(ts,speed,fdetail);
			
			t += ts;
		}
		
		/* same as the previous, tw = ts case */
		void advance(unsigned int ts, bool treserve, unsigned int max_wait_time,
				double speed, FILE* fdetail, FILE* fout = 0, bool limit_new = false) {
			advance(ts,ts,treserve,max_wait_time,speed,fdetail,fout,limit_new);
		}
		
		bool is_end() const { return ix1 >= start_events.size() && end_events.empty(); }
};


int main(int argc, char **argv)
{
	unsigned int ts = 300;
	unsigned int tw = 300;
	unsigned int max_wait_time = 300;
	char* trips = 0;
	//~ char* network_fn = 0;
	double speed = 5000.0 / 3600.0;
	char* detail_out = 0;
	bool treserve = false;
	unsigned int max_vehicles = 0;
	bool limit_new = false;
	
	char* trip_links_fn = 0;
	char* node_links_fn = 0;
	char* init_dist_fn = 0; /* file with initial distribution of vehicles */
	bool links_zip = false; /* if true, trip and node links input files are compressed with gzip */
	
	bool precise_sort = false;
	
	unsigned int random_init = 0; /* randomly distribute this many vehicles before the start of calculation */
	std::mt19937 rng(time(0));
	
	for(int i=1;i<argc;i++) {
		if(argv[i][0] != '-') fprintf(stderr,"Unknown command line argument: %s!\n",argv[i]);
		else switch(argv[i][1]) {
			case 'i':
				init_dist_fn = argv[i+1];
				i++;
				break;
			case 'o':
				detail_out = argv[i+1];
				i++;
				break;
			case 't':
				trips = argv[i+1];
				i++;
				break;
			case 'T':
				ts = atoi(argv[i+1]);
				i++;
				break;
			case 'W':
				tw = atoi(argv[i+1]);
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
			case 'L':
				trip_links_fn = argv[i+1];
				i++;
				break;
			case 'N':
				node_links_fn = argv[i+1];
				i++;
				break;
			case 'p':
				precise_sort = true;
				break;
			case 'R':
				random_init = atoi(argv[i+1]);
				i++;
				break;
			case 'S':
				rng.seed(atoi(argv[i+1]));
				i++;
				break;
			case 'z':
				links_zip = true;
				break;
			default:
				fprintf(stderr,"Unknown command line argument: %s!\n",argv[i]);
				break;
		}
	}
	
	if( ! (trip_links_fn && node_links_fn) ) {
		fprintf(stderr,"Missing parameters!\n");
		return 1;
	}
	
	bikeshare_calculate p;
	{
		FILE* f_trip_links = 0;
		FILE* f_node_links = 0;
		if(links_zip) {
			size_t len = strlen(trip_links_fn);
			{
				size_t len2 = strlen(node_links_fn);
				if(len2 > len) len = len2;
			}
			len += 16;
			char* tmp = (char*)malloc(sizeof(char)*len);
			if(!tmp) {
				fprintf(stderr,"Error allocating memory!\n");
				return 1;
			}
			
			sprintf(tmp,"/bin/gzip -cd %s",trip_links_fn);
			f_trip_links = popen(tmp,"re");
			sprintf(tmp,"/bin/gzip -cd %s",node_links_fn);
			f_node_links = popen(tmp,"re");
			free(tmp);
		}
		else {
			f_trip_links = fopen(trip_links_fn,"r");
			f_node_links = fopen(node_links_fn,"r");
		}
		
		bool ret = true;
		if( !(f_trip_links && f_node_links) ) {
			fprintf(stderr,"Error opening input files!\n");
			ret = false;
		}
		if(ret) ret = p.read_events(read_table2(trips,stdin));
		
		if(ret) {
			p.max_vehicles = max_vehicles;
		
			read_table2 rt_trips(f_trip_links);
			read_table2 rt_nodes(f_node_links);
			rt_trips.set_fn(trip_links_fn);
			rt_nodes.set_fn(node_links_fn);
			if(!p.read_connections(rt_trips,rt_nodes,speed,max_wait_time,ts,precise_sort)) {
				fprintf(stderr,"Error reading trip connection links!\n");
				ret = false;
			}
		}
		else fprintf(stderr,"Error reading trips!\n");
		
		if(links_zip) {
			if(f_trip_links) pclose(f_trip_links);
			if(f_node_links) pclose(f_node_links);
		}
		else {
			if(f_trip_links) fclose(f_trip_links);
			if(f_node_links) fclose(f_node_links);
		}
		
		if(!ret) return 1;
	}
	
	FILE* fdetail = 0;
	if(detail_out) {
		fdetail = fopen(detail_out,"w");
		if(!fdetail) {
			fprintf(stderr,"Error opening output file %s!\n",detail_out);
			return 1;
		}
	}
	
	if(init_dist_fn) p.dist_file_init(read_table2(init_dist_fn));
	else if(random_init) p.random_init(random_init,rng);
	
	while(!p.is_end()) {
		p.advance(ts,tw,treserve,max_wait_time,speed,fdetail,stderr,limit_new);
		fflush(stderr);
		fprintf(stdout,"%u\t%u\t%u\n",p.get_ts(),p.get_num_vehicles(),p.get_trips_served());
		fflush(stdout);
	}
	
	return 0;
}

