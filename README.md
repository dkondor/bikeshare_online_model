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

