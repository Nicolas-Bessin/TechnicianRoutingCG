// Parse a JSON file to return a Instance object
#pragma once
#include <string>
#include "instance/instance.h"


Instance parse_file(std::string filename, std::string instance_name, bool verbose = false);