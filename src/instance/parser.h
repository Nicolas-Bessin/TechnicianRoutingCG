// Parse a JSON file to return a Instance object
#pragma once
#include <string>
#include "instance/instance.h"

// Parse a JSON file to return a Instance object
// @param filename : the name of the file to parse
// @param instance_name : the name of the instance
// @param n_interventions : the number of interventions to keep (if -1, keep all)
// @param verbose : whether to print the instance information
Instance parse_file(std::string filename, std::string instance_name, int n_interventions = -1, bool verbose = false);