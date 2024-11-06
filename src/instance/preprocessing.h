#pragma once

#include <vector>
#include "instance/instance.h"
#include "instance/constants.h"

// This function pre-processes the interventions to see if they are "ambiguous"
// i.e. if they can be done both fully in the morning and fully in the afternoon
// If not, we reduce the time window to the morning or the afternoon
// Print the number of interventions that we could reduce the time window for
void preprocess_interventions(Instance &instance, bool verbose = false);



// Checks wether an intervention is trivially non feasible
// To do this, we check the time window and the various capacities
// @param intervention
// @param instance
// @param available_vehicle - the list of vehicles that can be used for the intervention
// @return true if the intervention is trivially non feasible
bool is_trivially_non_feasible(const Node &intervention, const Instance &instance, const std::vector<int> &available_vehicle);


// Returns the maximum amount of intervention time that is a priori feasible, excluding the trivially non feasible interventions
int max_a_priori_feasible_time(const Instance &instance, bool verbose = false);