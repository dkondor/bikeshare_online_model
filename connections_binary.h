/*  -*- C++ -*-
 * connections_binary.h -- open binary file with connection edges
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
 */
 
#include <stdint.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>


template<class data_type>
class connections_binary {
	protected:
		void* ptr;
		const data_type* data;
		size_t n;
		size_t file_size;
		int f;
		
	public:
		connections_binary():ptr(MAP_FAILED),data(0),n(0UL),file_size(0UL),f(-1) { }
		explicit connections_binary(const char* fn):ptr(MAP_FAILED),data(0),n(0UL),file_size(0UL),f(-1) {
			if(!open(fn)) throw std::runtime_error("connections_binary(): error opening file!\n");
		}
		~connections_binary() { close(); }
		
		bool open(const char* fn) {
			close();
			if(!fn) return false;
			f = ::open(fn,O_CLOEXEC | O_RDONLY);
			if(f == -1) return false;
			{
				struct stat st;
				int r = fstat(f,&st);
				if(r) { close(); return false; }
				file_size = st.st_size;
				if(file_size < 2*sizeof(uint64_t)) { close(); return false; }
			}
			ptr = mmap(0,file_size,PROT_READ,MAP_SHARED,f,0);
			uint64_t* p = (uint64_t*)ptr;
			if(*p != data_type::magic_number) { close(); return false; }
			++p;
			n = *p;
			
			size_t total_size = n*sizeof(data_type) + 2*sizeof(uint64_t);
			if(total_size != file_size) { close(); return false; }
			
			data = (data_type*)((char*)ptr + 2*sizeof(uint64_t));
			return true;
		}
		
		void close() {
			if(ptr != MAP_FAILED) {
				munmap(ptr,file_size);
				ptr = MAP_FAILED;
			}
			if(f != -1) {
				::close(f);
				f = -1;
			}
			n = 0UL;
			file_size = 0UL;
			data = 0;
		}
		
		const data_type* begin() const { return data; }
		const data_type* end() const { return data + n; }
		const data_type* cbegin() const { return data; }
		const data_type* cend() const { return data + n; }
};


