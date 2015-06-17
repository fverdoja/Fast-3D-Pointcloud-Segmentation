/*
 * ClusteringState.cpp
 *
 *  Created on: 01/giu/2015
 *      Author: Francesco Verdoja <verdoja@di.unito.it>
 */

#include "ClusteringState.h"

ClusteringState::ClusteringState(ClusteringT s, WeightMapT w) {
	set_segments(s);
	set_weight_map(w);
}
