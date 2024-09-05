#pragma once

#include "instance/instance.h"

#include <vector>


// Computes the Hamming distance between every pair of vehicles
// Distance is defined by the "difference" in the interventions each vehicle can do
// Each intervention that can only be done by one of the vehicles contributes 1 to the distance
int hamming_distance(const Vehicle& vehicle1, const Vehicle& vehicle2);