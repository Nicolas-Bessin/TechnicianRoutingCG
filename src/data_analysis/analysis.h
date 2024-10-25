#pragma once

#include "instance/instance.h"
#include "master_problem/master.h"
#include "routes/route.h"

#include <vector>

// Count the number of zeros in a vector
int count_zeros(const std::vector<double>& vector);

// Count the "number" of interventions covered by the solution (i.e. a_ir x_r)
double count_covered_interventions(const MasterSolution& solution, const std::vector<Route>& routes, const Instance& instance);

// Returns a vector of size n_interventions, with a 1 at the index of each intervention that is covered by the solution
std::vector<int> covered_interventions(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance, bool details = false);

int count_covered_interventions(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

// Returns a vector of size n_vehicle, with a 1 at the index of each vehicle that is used in the solution
std::vector<int> used_vehicles(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

double relaxed_solution_cost(const MasterSolution& solution, const std::vector<Route>& routes);

// Returns the fixed cost in a given solution
double fixed_cost(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

int count_used_vehicles(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

// Returns a pair : the first element is the fractionnal number of vehicles used, the second is the number unique vehicles used
std::pair<double, int> count_used_vehicles(const MasterSolution& solution, const std::vector<Route>& routes, const Instance& instance);

void print_used_vehicles(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

double time_spent_travelling(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

double time_spent_working(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

double time_spent_waiting(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

// Returns the used route which has the shortest cumulative time
int shortest_time_route(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

// Returns the used route which has the longest cumulative time
int longest_time_route(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

void print_used_route_durations(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

// Count the number of interventions the vehicles used in the solution can perform
// (without considering the time windows, only the skills)
int count_coverable_interventions(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

int count_routes_with_duplicates(const std::vector<Route>& routes);

int count_used_routes_with_duplicates(const IntegerSolution& solution, const std::vector<Route>& routes);

double count_kilometres_travelled(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);



// Prints a route to the console
void print_route(const Route& route, const Instance& instance, const MasterSolution& solution = MasterSolution());

// Print the used routes in a solution
void print_used_routes(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

// Print the interventions not covered by the solution
void print_non_covered_interventions(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance, bool details = false);

// Print the vehicles that can cover all the interventions not covered by the solution
void print_vehicles_non_covered(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance);

// Full analysis of the solution
void full_analysis(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance, bool details = false);

