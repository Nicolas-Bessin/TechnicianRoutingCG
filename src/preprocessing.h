#pragma once

#include <vector>
#include "instance.h"
#include "constants.h"

// This function pre-processes the interventions to see if they are "ambiguous"
// i.e. if they can be done both fully in the morning and fully in the afternoon
// If not, we reduce the time window to the morning or the afternoon
// Print the number of interventions that we could reduce the time window for
void preprocess_interventions(Instance &instance, bool verbose = false);