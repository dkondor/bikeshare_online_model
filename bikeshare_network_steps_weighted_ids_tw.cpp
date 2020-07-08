/*
 * bikeshare_network_steps_weighted_ids_tw.cpp
 * 
 * on-line simulation of autonomous scooter fleet operation with extra
 * look-ahead window
 * 
 * Copyright 2020 Daniel Kondor <kondor.dani@gmail.com>
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


/*
 * maximum matching is performed step-by-step -- new step is calculated every t_1 minutes
 * 
 * in each step, events (trip start and ends) are considered and a network is created
 * 
 * additionally: keep track of trip IDs, it is possible to keep track of trip chains
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

#include <lemon/smart_graph.h>
#include <lemon/matching.h>


/* struct to hold "events" -- an event is either the start or the end
 * of a trip */
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
		/* trip ends, sorted by time -- use vector to be able to look up events (for unweighted matching) */
		std::vector<trip_event> end_events;
		//~ std::set<trip_event> end_events;
		/* delays for trip ends -- checked when processing trip end event */
		std::unordered_map<unsigned int,unsigned int> trip_end_delay;
		/* get delay or 0 if not present in the map */
		unsigned int get_end_delay(unsigned int id) const {
			auto it = trip_end_delay.find(id);
			if(it == trip_end_delay.end()) return default_delay;
			return it->second;
		}
		
		/* start events (index) sorted by node ID and time */
		std::vector<size_t> nix;
		/* index (by node ID) into the previous */
		std::unordered_map<uint64_t,std::pair<size_t,size_t> > nix2;
		
		/* store trip start and end events that have been matched */
		std::unordered_set<unsigned int> matched_start;
		std::unordered_set<unsigned int> matched_end;
		/* end events where the vehicle have been parked already
		 * (so should be ignored) */
		std::unordered_set<unsigned int> parked_end;
		
		/* current (simulation) time */
		unsigned int t;
		/* current index in events */
		size_t ix1; /* for trip starts */
		size_t ix_end; /* for trip ends */
		node_distances nd;
		spindex_adapter<uint16_t> spindex;
		
		/* total parking -- also used to get IDs for new parking spaces */
		unsigned int bikes_total;
		unsigned int bikes_counter; /* temp counter to use as IDs in nodes_vehicles */
		unsigned int trips_served; /* number of trips succesfully served */
		
		unsigned int time0; /* timestamp for start of calculations */
	
	public:
		explicit bikeshare_calculate() : t(0), ix1(0), bikes_total(0U), time0(0U), bikes_counter(0U), trips_served(0U),
			max_vehicles(0U), only_matched_end(false), default_delay(0U) { }
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
		/* if true, only use end events where the corresponding start event has been processed already */
		bool only_matched_end;
		/* default value for trip end delay */
		unsigned int default_delay;
		
		bool read_events(read_table2&& rt) {
			start_events.clear();
			end_events.clear();
			matched_start.clear();
			matched_end.clear();
			parked_end.clear();
			t = 0;
			ix1 = 0;
			ix_end = 0;
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
				end_events.push_back(trip_event{node2,dist_end,ts2,id});
				if(start_events.size() > UINT32_MAX || end_events.size() > UINT32_MAX) {
					fprintf(stderr,"bikeshare_calculate::read_events(): too many events!\n");
					start_events.clear();
					end_events.clear();
					return false;
				}
			}
			if(rt.get_last_error() != T_EOF || start_events.size() == 0) {
				fprintf(stderr,"bikeshare_calculate::read_events(): ");
				rt.write_error(stderr);
				start_events.clear();
				end_events.clear();
				return false;
			}
			std::sort(start_events.begin(),start_events.end());
			std::sort(end_events.begin(),end_events.end());
			
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
			operator std::pair<unsigned int,unsigned int>() const { return std::make_pair(first,second); }
		};
		*/
		
		/* general sentinel for all dynamic iterators to signal end of iteration */
		struct iterator_sentinel { };
		
		/* iterator for creating shareability among trips */
		template<class spindex_type>
		struct trips_graph_iterator {
			const bikeshare_calculate& p;
			size_t ix; /* current index in end_events */
			unsigned int tmax; /* maximum timestamp to consider for trip end events */
			unsigned int tmax_start; /* maximum timestamp to consider for trip start events */
			bool is_end() const { return ix == p.end_events.size() || p.end_events[ix].ts >= tmax; }
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
			/* delay for current trip end event (pointed to by ix) */
			unsigned int delay;
			
			trips_graph_iterator(const bikeshare_calculate& p_, const spindex_type& spindex_,
				unsigned int tmax_, unsigned int tmax_start_,
				const std::unordered_set<unsigned int>& exclude_matched_, double speed_,
				unsigned int max_connection_time_, unsigned int max_wait_time_)
					:p(p_),tmax(tmax_),tmax_start(tmax_start_),exclude_matched(exclude_matched_),
					spindex(spindex_),speed(speed_),max_connection_time(max_connection_time_),
					max_wait_time(max_wait_time_) {
				ix = p.ix_end;
				while ( ! (is_end() || new_ix()) ) ++ix;
			}
			void operator++() { advance(); }
			void operator++(int) { advance(); }
			const edge& operator *() const { return res; }
			const edge* operator ->() const { return &res; }
			
			bool new_ix() {
				const trip_event& ee = p.end_events[ix];
				if(p.matched_end.count(ee.id)) return false;
				if(p.parked_end.count(ee.id)) return false;
				if(p.only_matched_end) if(!exclude_matched.count(ee.id)) return false;
				delay = p.get_end_delay(ee.id);
				if(ee.ts + delay >= tmax) return false;
				res.first = (unsigned int)ix;
				it = spindex.begin(ee.n);
				it_end = spindex.end(ee.n);
				return new_i();
			}
			
			bool new_i() {
				const trip_event& ee = p.end_events[ix];
				for(;it!=it_end;++it) {
					connection_time = (unsigned int)floor((round(it->d) + ee.node_dist) / speed);
					if(max_connection_time && connection_time > max_connection_time) return false;
					uint64_t n = it->node_id;
					
					{
						auto it = p.nix2.find(n);
						if(it == p.nix2.end()) continue;
						start_ix = it->second.first;
						end_ix = it->second.second;
					}
					
					t2 = ee.ts + delay + connection_time;
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
				const trip_event& ee = p.end_events[ix];
				if(exclude_matched.count(p.start_events[ p.nix[j] ].id)) return false;
				t2 = ee.ts + delay + (unsigned int)ceil( (p.start_events[p.nix[j]].node_dist + round(it->d) + ee.node_dist) / speed);
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
					res.second = (unsigned int)p.nix[j];
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
					unsigned int connection_time = (unsigned int)ceil( (round(it->d) + p.start_events[ix].node_dist) / speed);
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
		
		
		
		/* process end_events between t and t+ts, assume vehicles park at the closest node
		 * 
		 * change from previous version: do not adjust trip end time, only remove processed events
		 */
		void process_end_events(unsigned int ts, double speed, FILE* fdetail) {
			for(size_t ix2 = ix_end; ix2 < end_events.size(); ++ix2) {
				const trip_event& ee = end_events[ix2];
				if(ee.ts >= t + ts) break;
				unsigned int trip_id = ee.id;
				if(matched_end.count(trip_id) || parked_end.count(trip_id)) {
					/* increase minimum possible index if all events up to now has been processed */
					if(ix2 == ix_end) ix_end++;
				}
				else {
					uint64_t n = ee.n;
					unsigned int t1 = ee.ts;
					unsigned int t2 = t1 + (unsigned int)ceil(ee.node_dist / speed);
					
					unsigned int delay = get_end_delay(trip_id);
					t2 += delay;
					/* do not adjust yet if the vehicle cannot reach the "node" exactly before t + ts */
					if(t2 >= t + ts) continue;
					
					/* here t2 + delay < t + ts, we need to "park" the vehicle at node n */
					if(ix2 == ix_end) ix_end++;
					parked_end.insert(trip_id);
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
		 * tw parameter gives a look-ahead window for events to try to match
		 * treserve parameter gives if trips are reserve in advance:
		 * if it's true, we assume that the current time is t, and we know all trip reservations in [t,t+ts]
		 * otherwise, we assume that the current time is t+ts and we pooled reservations since t */
		void advance(unsigned int ts, unsigned int tw, bool unweighted, bool match_end_time, bool treserve,
				unsigned int max_wait_time, double speed, FILE* fdetail, FILE* fout = 0, bool limit_new = false) {
			
			if(!only_matched_end) default_delay = max_wait_time;
			
			/* 0. update trip end times and remove matched end events if needed */
			/* if the "current time" is t + ts, we need to process end_events that happen before this time
			 * otherwise, this processing happens at the end of this function
			 * in both cases, in the next step, end_events only contains "valid" events */
			if(!treserve) process_end_events(ts,speed,fdetail);
			
			/* temporary set for matched start events */
			//~ std::set<unsigned int> matched_start_tmp(matched_start);
			
			/* 1. create shareability network among the trips 
					trips_graph_iterator(const bikeshare_calculate& p_, unsigned int tmax_, unsigned int tmax_start_,
				const std::unordered_set<unsigned int>& exclude_matched_, double speed_,
				unsigned int max_connection_time_, unsigned int max_wait_time_) */
			{
				std::vector<edge> matching;
				std::vector<std::pair<unsigned int, unsigned int> > unweighted_matching;
				if(fout) fprintf(fout,"%u, calculating matching among trips, ",(unsigned int)time(0)-time0);
				
				if(spindex.is_sorted()) {
					trips_graph_iterator<spindex_adapter<uint16_t> > it(*this, spindex,
						t + tw, t + tw, matched_start, speed, 0, max_wait_time);
					if(unweighted) matching_helper::calculate_matching_unweighted(std::move(it), 
						iterator_sentinel(), unweighted_matching, fout);
					else matching_helper::calculate_matching(std::move(it),
						iterator_sentinel(), matching, max_wait_time, fout);
				}
				else {
					trips_graph_iterator<node_distances> it(*this, nd, t + tw, t + tw,
						matched_start, speed, 0, max_wait_time);
					if(unweighted) matching_helper::calculate_matching_unweighted(std::move(it), 
							iterator_sentinel(), unweighted_matching, fout);
					else matching_helper::calculate_matching(std::move(it),
						iterator_sentinel(), matching, max_wait_time, fout);
				}
								
				/* process these matches */
				size_t nmatches = unweighted ? unweighted_matching.size() : matching.size();
				for(size_t i = 0; i < nmatches; i++) {
					std::pair<unsigned int, unsigned int> x = unweighted ? unweighted_matching[i] : matching[i];
					/* x.first is the index into the end events array,
					 * x.second is the index into the start events array that has been matched */
					unsigned int start_id = start_events[x.second].id;
					unsigned int start_time = start_events[x.second].ts;
					unsigned int end_id = end_events[x.first].id;
					unsigned int end_time = end_events[x.first].ts;
					unsigned int delay = 0;
					
					if(unweighted) {
						/* We have to calculate all the travel times again,
						 * since these were "lost" when creating the graph.
						 * 
						 * A better implementation would save the weights in the
						 * graph and keep track of them. */
						const trip_event& se = start_events[x.second];
						const trip_event& ee = end_events[x.first];
						
						/* calculate travel time */
						uint64_t end_node = ee.n;
						uint64_t start_node = se.n;
						double connection_dist;
						if(spindex.is_sorted()) connection_dist = spindex.get_dist(end_node, start_node);
						else connection_dist = nd.get_dist(end_node, start_node); /* this is not implemented, will throw exception */
						double travel_dist = round(connection_dist) + se.node_dist + ee.node_dist;
						unsigned int travel_time = (unsigned int)ceil(travel_dist / speed);
						unsigned int t2 = end_time + get_end_delay(end_id) + travel_time;
						if(t2 > start_time) delay = t2 - start_time;
					}
					else delay = matching[i].dist;
					
					if(delay > 0) trip_end_delay[start_id] = delay;
					
					/* only use the matches where the start event time is under t + ts
					 * OR end time is under t + ts */
					if(start_time < t + ts || (match_end_time && end_time < t + ts)) {
						matched_end.insert(end_id);
						matched_start.insert(start_id);
						trips_served++;
						
						/* write out matching */
						if(fdetail) fprintf(fdetail,"1\t%u\t%u\t%u\n",end_id,start_id,delay);
					}
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
				
				/* here, we only match trip starts before t + ts -- we could reserve vehicles for later here as well */
				if(spindex.is_sorted()) {
					trip_start_iterator<spindex_adapter<uint16_t> > it(*this, spindex,
						ix1, t + ts, t1, max_wait_time, speed, matched_start);
					matching_helper::calculate_matching(std::move(it),
						iterator_sentinel(), start_matching, max_wait_time, fout);
				}
				else {
					trip_start_iterator<node_distances> it(*this, nd,
						ix1, t + ts, t1, max_wait_time, speed, matched_start);
					matching_helper::calculate_matching(std::move(it),
						iterator_sentinel(), start_matching, max_wait_time, fout);
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
				if(max_wait_time == 0) delay = 0; /* simplified case, we require all trips to be served without waiting */
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
			if(treserve) process_end_events(ts,speed,fdetail);
			
			t += ts;
		}
	
		bool is_end() const { return ix1 >= start_events.size() && ix_end >= end_events.size(); }
};


int main(int argc, char **argv)
{
	unsigned int ts = 300;
	unsigned int max_wait_time = 300;
	unsigned int tw = 600;
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
	bool unweighted_match = false; /* if true, use unweighted instead of weighted matching */
	bool only_matched_end = false; /* if true, only match end events whose corresponding start has been processed already (for compatibility with previous version) */
	bool match_end_time = true; /* if true, end events between t and t + ts that have a match will be processed
						in each step even if their match (in start events) is later; if false, only start events */
	
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
			case 'u':
				unweighted_match = true;
				break;
			case 'E':
				only_matched_end = true;
				break;
			case 'e':
				match_end_time = false;
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
	p.only_matched_end = only_matched_end;
	
	FILE* fdetail = 0;
	if(detail_out) {
		fdetail = fopen(detail_out,"w");
		if(!fdetail) {
			fprintf(stderr,"Error opening output file %s!\n",detail_out);
			return 1;
		}
	}
	
	while(!p.is_end()) {
		p.advance(ts,tw,unweighted_match,match_end_time,treserve,max_wait_time,speed,fdetail,stderr,limit_new);
		fflush(stderr);
		fprintf(stdout,"%u\t%u\t%u\n",p.get_ts(),p.get_num_vehicles(),p.get_trips_served());
		fflush(stdout);
	}
	
	return 0;
}

