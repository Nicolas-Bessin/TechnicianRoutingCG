// Parse a JSON file to return a Instance object
#pragma once
#include <string>
#include "instance.h"


Instance parse_file(std::string filename, bool verbose = false);