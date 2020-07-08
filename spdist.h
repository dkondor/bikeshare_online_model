/*  -*- C++ -*-
 * spdist.h -- spatial distances and index of these
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


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <vector>
#include <limits>
#include <utility>
#include <algorithm>
#include <stdexcept>
#include "read_table.h"
#include "mmap.h"




/* include a mutex for modifying file mappings */
#ifdef USE_MUTEX
#include <mutex>
#endif

template<class ixtype>
class sp_index {
	protected:
		unsigned int matrix_size;
		
		FileMappingT<ixtype> dists;
		
#ifdef USE_MUTEX
		std::recursive_mutex mutex1;
		void lock_mutex() { mutex1.lock(); }
		void unlock_mutex() { mutex1.unlock(); }
#else
		void lock_mutex() { }
		void unlock_mutex() { }
#endif
		
		/* get travel distance between known nodes */
		ixtype get_dist(unsigned int start, unsigned int end) const {
			uint64_t tmp = matrix_size;
			uint64_t offset = start*tmp + end;
			return dists[offset];
		}
		
	public:
		
		sp_index():matrix_size(0) { }
		
		unsigned int get_matrix_size() const { return matrix_size; }
		
		/* open index */
		bool open(const char* fn) {
			/* make sure no files are open */
			lock_mutex();
			close();
			bool ret = dists.open_file(fn);
			if(ret) ret = dists.map_file();
			if(ret) {
				uint64_t size1 = dists.size();
				uint64_t size2 = (unsigned int)floor(sqrt((double)size1));
				if(size2*size2 != size1) {
					fprintf(stderr,"sp_index::open(): invalid file size!\n");
					close();
					ret = false;
				}
				else matrix_size = (unsigned int)size2;
			}
			unlock_mutex();
			return ret;
		}
		
		/* close open file */
		void close() {
			lock_mutex();
			dists.close_file();
			unlock_mutex();
		}
		
		
		/* Get travel time between two points using a query with the IDs
		 * Returns 0 on success, 1 on error */
		bool get_dist_ix(unsigned int start_id, unsigned int end_id, ixtype& dist) const {
			if(!dists.is_mapped()) {
				fprintf(stderr,"sp_index::get_dist_ix(): index file not opened!\n");
				return false;
			}
			if(start_id >= matrix_size || end_id >= matrix_size) {
				fprintf(stderr,"sp_index::get_dist_ix(): too large IDs: %u %u!\n",start_id,end_id);
				return false;
			}
			
			dist = get_dist(start_id,end_id);
			return true;
		}
		
		ixtype get_dist_ix(unsigned int start_id, unsigned int end_id) const {
			ixtype dist1;
			if(!get_dist_ix(start_id, end_id, dist1)) return std::numeric_limits<ixtype>::max();
			else return dist1;
		}
		
		unsigned int size() const { return matrix_size; }
};


/* sort IDs according to distance for efficient search */
template<class ixtype, bool symmetric = true>
class sp_sorted_index : public sp_index<ixtype> {
	protected:
		/* type for indexing in the sorted matrix */
		typedef uint32_t ixtype2;
		
		/* index from a file saved previously or allocated in memory for this instance */
		FileMappingT<ixtype2> file_sorted;
		
	public:
		sp_sorted_index() { }
		
		/* create sorted index, either saving it to a file or
		 * only in memory if fn == 0 */
		bool create_sorted(const char* fn = 0) {
			this->lock_mutex();
			file_sorted.close_file();
			if(fn) {
				if(!file_sorted.open_file(fn, FileMapping::Mode::ReadWrite, true)) {
					this->unlock_mutex();
					return false;
				}
			}
			else file_sorted.open_anon();
			
			uint64_t filesize = this->size();
			filesize = filesize * filesize * sizeof(ixtype2);
			if(!symmetric) filesize *= 2;
			
			if(!file_sorted.change_file_size(filesize)) {
				this->unlock_mutex();
				return false;
			}
			
			if(!file_sorted.map_file(FileMapping::Mode::ReadWrite, true)) {
				this->unlock_mutex();
				return false;
			}
			
			/* opened file successfully / allocated memory, create the sorted index */
			for(int k=0;k<2;k++) {
				/* we need two iterations, first for from distances, second for to distances */
				if(k && symmetric) break;
				uint64_t size1 = this->size();
				ixtype2* matrix = file_sorted.data();
				const ixtype* dist_matrix = this->dists.data();
				if(k) matrix += size1*size1;
				for(ixtype2 i=0;i<this->size();i++) {
					for(ixtype2 j=0;j<this->size();j++) matrix[i*size1 + j] = j;
					auto sorter = [dist_matrix, i, k, size1](ixtype2 x, ixtype2 y) {
						ixtype dist1, dist2;
						if(k) {
							/* k == 1, to distances */
							dist1 = dist_matrix[x*size1 + i];
							dist2 = dist_matrix[y*size1 + i];
						}
						else {
							/* k == 0, from distances */
							dist1 = dist_matrix[i*size1 + x];
							dist2 = dist_matrix[i*size1 + y];
						}
						if(dist1 == dist2) {
							/* ensure that "travel" between the same locations is always the
							 * shortest even if there are zero distances to other locations */
							if(i == x && i != y) return true;
							if(i != x && i == y) return false;
						}
						return dist1 < dist2;
					};
					std::sort(matrix + i*size1, matrix + (i+1)*size1, sorter);
				}
			}
			/* change the memory to read-only */
			file_sorted.set_mem_mode(FileMapping::Mode::ReadOnly);
			this->unlock_mutex();
			return true;
		}
		
		
		/* open an already exsting sorted file */
		bool open_sorted_file(const char* fn) {
			this->lock_mutex();
			if(!this->dists.is_mapped()) {
				fprintf(stderr,"sp_sorted_index::open_sorted_file_nolock(): need to open distance matrix first!\n");
				this->unlock_mutex();
				return false;
			}
			
			bool ret = file_sorted.open_file(fn);
			if(ret) ret = file_sorted.map_file();
			if(ret) {
				uint64_t size1 = file_sorted.size();
				if(symmetric) {
					if(size1 != this->dists.size()) {
						fprintf(stderr,"sp_sorted_index::open_sorted_file_nolock(): invalid matrix size!\n");
						file_sorted.close_file();
						ret = false;
					}
				}
				else {
					if(size1 / 2 != this->dists.size()) {
						fprintf(stderr,"sp_sorted_index::open_sorted_file_nolock(): invalid matrix size!\n");
						file_sorted.close_file();
						ret = false;
					}
				}
			}
			this->unlock_mutex();
			return ret;
		}
		
		/* close sorted file */
		void close_sorted_file() {
			this->lock_mutex();
			file_sorted.close_file();
			this->unlock_mutex();
		}
		
		/* close all files */
		void close() {
			this->lock_mutex();
			close_sorted_file();
			sp_index<ixtype>::close();
			this->unlock_mutex();
		}
		
		
		typedef const ixtype2* const_iterator;
		const_iterator begin_from(unsigned int id) const {
			if(!file_sorted.is_mapped()) return nullptr;
			if(id >= this->size()) return nullptr;
			const ixtype2* ptr = file_sorted.data();
			uint64_t size1 = this->size();
			return ptr + size1*id;
		}
		const_iterator end_from(unsigned int id) const {
			if(!file_sorted.is_mapped()) return nullptr;
			if(id >= this->size()) return nullptr;
			const ixtype2* ptr = file_sorted.data();
			uint64_t size1 = this->size();
			return ptr + size1*(id+1);
		}
		const_iterator begin_to(unsigned int id) const {
			if(!file_sorted.is_mapped()) return nullptr;
			if(id >= this->size()) return nullptr;
			const ixtype2* ptr = file_sorted.data();
			uint64_t size1 = this->size();
			if(!symmetric) ptr += size1*size1;
			return ptr + size1*id;
		}
		const_iterator end_to(unsigned int id) const {
			if(!file_sorted.is_mapped()) return nullptr;
			if(id >= this->size()) return nullptr;
			const ixtype2* ptr = file_sorted.data();
			uint64_t size1 = this->size();
			if(!symmetric) ptr += size1*size1;
			return ptr + size1*(id+1);
		}
		
		template<bool symmetric_ = symmetric>
		typename std::enable_if<symmetric_, const_iterator>::type
		begin(unsigned int id) const { return begin_from(id); }
		
		template<bool symmetric_ = symmetric>
		typename std::enable_if<symmetric_, const_iterator>::type
		end(unsigned int id) const { return end_from(id); }
		
		void close_sorted() {
			file_sorted.close_file();
		}
		
		bool is_sorted() const { return file_sorted.is_mapped(); }
};




