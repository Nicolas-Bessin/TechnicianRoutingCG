// Include the gurobi api header
#pragma once
#include "solution.h"

MasterSolution cg_solver(Instance instance, double time_limit);