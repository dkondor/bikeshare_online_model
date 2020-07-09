# bikeshare_online_model
Code used for the online model in paper https://arxiv.org/abs/1909.03679

This repository contains four version of the online model:

 - bikeshare_network_weighted_ids.cpp: basic version, new vehicles are
	added on the fly to make sure all trips are served
 - bikeshare_network_weighted_ids2.cpp: alternate implementation,
	distances from all trip ends to all trip starts need to be
	precalculated up to a given radius to speed up processing
 - bikeshare_network_weighted_ids_nomax.cpp: version with unlimited
	waiting times, no new vehicles are added during the day, instead
	they are distributed randomly in the beginning; needs precalculated
	distances as well
 - bikeshare_network_weighted_ids_tw.cpp: limited oracle model, upcoming
	trips requests are assumed to be known in a T_{LA} time window in
	advance, optimization is still performed in batches

## Compilation

This code was tested on Ubuntu 18.04 with gcc 7.5.0. It should work on
other systems as well with a C++14 compiler. It also requires the LEMON
library for graph calculations, available [here](https://lemon.cs.elte.hu/)
or as the `liblemon-dev` package on Ubuntu.

The programs can be compiled with the following commands (assuming gcc):

```
g++ -o bikeshare_network_steps_weighted_ids_tw bikeshare_network_steps_weighted_ids_tw.cpp graph.cpp -O3 -march=native -std=gnu++14 -lm -llemon
g++ -o bikeshare_network_steps_weighted_ids bikeshare_network_steps_weighted_ids.cpp -std=gnu++14 -O3 -march=native -lm -llemon
g++ -o bikeshare_network_steps_weighted_ids2 bikeshare_network_steps_weighted_ids2.cpp -std=gnu++14 -O3 -march=native -lm -llemon
g++ -o bikeshare_network_steps_weighted_ids_nomax bikeshare_network_steps_weighted_ids_nomax.cpp -std=gnu++14 -O3 -march=native -lm -llemon
```

## Inputs

1. List of preprocessed trips, matched to the nodes in the OSM network,
	e.g. https://github.com/dkondor/bikesharing_data/blob/master/bike_trips/bike_trips2_nodes_distances_20170911.dat

2. Path network used for navigation: https://github.com/dkondor/bikesharing_data/blob/master/osm/sg_osm_edges.dat

3. Pre-computed distances between nodes in binary format (optional,
	for bikeshare_network_steps_weighted_ids and bikeshare_network_steps_weighted_ids_tw; 
	calculate with the helper program sp_distance_matrix_p, see below)

4. Optionally, a separate list of edges that have been improved, allowing 
	faster travel.

5. A precomputed list of distances between trip ends and starts and
	distances between OSM nodes and trip starts (for bikeshare_network_steps_weighted_ids2
	and bikeshare_network_steps_weighted_ids_nomax, to be calculated
	separately, not included here).


## Running

### Common parameters

| Option | Meaning |
| ------ | ------- |
| `-t`   | Input file name containing trips. Optional, if not given, trips are read from standard input. |
| `-o`   | File name for optional, detailed output. See the description of the format below. |
| `-T`   | Batch time window size, in seconds. Time is advanced in this big steps. |
| `-r`   | Whether trips are assumed to be reserved in advance. If given, it is assumed that trip requests are collected in time windows of size given by the `-T` option and then processed together. If not given, it is assumed that the central controller can predict trips in advance in the given time window. |
| `-w`   | Maximum waiting time acceptable for passengers, in seconds. |
| `-s`   | Speed of vehicles during relocation in km/h. |
| `-m`   | Maximum number of vehicles; new vehicles are only added if the total number would not exceed this. 0 means unlimited. If used, some of the trips might go unserved or be served only with long delays. |


### Parameters for bikeshare_network_steps_weighted_ids and bikeshare_network_steps_weighted_ids_tw

| Option | Meaning |
| ------ | ------- |
| `-n`   | Path network file. Only used if pre-computed distances are not provided. |
| `-i`   | File name with a list of improved edges (optional). |
| `-Í`   | Weight of improved edges. It is assumed that travel on improved edges is faster by this factor. |
| `-d`   | File names of the binary index files created with sp_distance_matrix_p. Need to supply three file names: distances, index and remapping of IDs; all of these are generated in one step, see below. |

### Parameters only used for bikeshare_network_steps_weighted_ids

| Option | Meaning |
| ------ | ------- |
| `-Ws`  | Walking speed, in km/h. If set, passengers are willing to walk to meet a vehicle before starting their trip. |
| `-Wd`  | Maximum walking distance in meters that passengers are willing to go to meet their vehicle at the start of a trip. Default is 0, i.e. no walking. |

### Parameters only used for bikeshare_network_steps_weighted_ids

| Option | Meaning |
| ------ | ------- |
| `-W`   | Look-ahead time, in seconds. If given, it is assumed that the operator knows all trips that will happen in a time window given by this. This is used to run the "limited oracle" model of the paper. |
| `-u`   | If given, unweighted matching is used to assign vehicles to trip requests (by default, a weighted matching is used that minimizes total waiting time). This can be used to speed up processing in this case considerably. |

### Parameters for bikeshare_network_steps_weighted_ids2 and bikeshare_network_steps_weighted_ids_nomax

| Option | Meaning |
| ------ | ------- |
| `-L`   | File name to read the precomputed links between trips from. |
| `-N`   | File name to read the precomputed links between OSM nodes and trips starts from. |
| `-z`   | Optional, means that the previous two files are compressed by gzip. |
| `-b`   | Optional, means that the previous two files are in a binary format. |
| `-i`   | Input file specifying an initial distribution of vehicles to use. |
| `-R`   | Number of vehicles to place in random locations initially. |
| `-S`   | Seed to use for random number generator. |


## Output

Main output is written on the standard output, and it shows the current time of day, number of vehicles and number of trips served after each batch step.

Additionally, diagnostic messages are written on the standard error.

More detailed output for each trip event can be written to a separate file if the `-o` option is used. This file contains the following columns:
```
event_type	id1	id2	delay
```
The meaning of the event types is the following:
| `event_type` | meaning |
| ------------ | ------- |
| 1 | Direct matching between the end of a trip and the start of a new trip (i.e. a vehicle is assigned before the end of its trip). `id1` is the ID of the ending trips and `id2` is the ID of the next starting trip. |
| 2 | An idle vehicle is assigned to a trip request. `id1` is the ID of the trip request and `id2` is the OSM ID of the node where the vehicle was parked before. |
| 3 | A trip ends without the vehicle assigned to a new trip yet. `id1` is the ID of the trip ending and `id2` is the OSM ID of the node where the vehicle parks. |

In all cases, `delay` is the delay experienced by the user starting the trip, in seconds. For `event_type == 3`, it is always 0 (start of the trip with the same ID can be used to establish it).


## Preprocessing node distances

This is done by a separate program. It can be compiled with the following command:
```
g++ -o sp_distance_matrix_p sp_distance_matrix_p.cpp -O3 -march=native -std=gnu++11 -lm -pthread
```
And run as following:
```
./sp_distance_matrix_p -T 16 -o sg_osm_dist16.bin -s sg_osm_dist16_index.bin -t uint16 < osm/sg_osm_nodes.dat > sg_osm_dist16_nodes.dat
```
It has the following command line parameters:
| Option | Meaning |
| ------ | ------- |
| `-n`   | Path network file. Optional, if not given, the network is read from standard input. |
| `-o`   | Main output file: distances in binary format between all nodes. |
| `-s`   | Output file for a sorted index for the previous in a binary format. |
| `-t`   | Data type to use for distances. Valid values are uint16, uint32, uint64, float and double. For integer outputs, the results are rounded and checked for overflow. Smaller data types will have smaller file sizes. |
| `-T`   | Number of threads to use for the main calculation (default is 1). |
| `-D`   | Output status to stderr after processing this many start nodes. |

Mapping of OSM IDs to the IDs used in the resulting matrix are written additionally on the standard output and saved to a file as well. All three output files should be then given to the online simulation programs.




