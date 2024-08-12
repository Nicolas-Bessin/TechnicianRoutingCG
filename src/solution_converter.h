#pragma once

#include "instance.h"
#include "master.h"
#include "compact_solver.h"

#include <vector>


std::vector<Route> compact_solution_to_routes(const Instance& instance, const CompactSolution<int>& compact_solution);

CompactSolution<double> to_compact_solution(const MasterSolution& master_solution, const std::vector<Route>& routes, const Instance& instance);

CompactSolution<int> to_compact_solution(const IntegerSolution& integer_solution, const std::vector<Route>& routes, const Instance& instance);