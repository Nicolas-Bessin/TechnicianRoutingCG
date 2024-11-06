#pragma once

#include "instance/instance.h"
#include "master_problem/master.h"
#include "routes/route.h"
#include "master_problem/node.h"

#include "gurobi_c++.h"

#include <vector>


// - Creates a Gurobi model from an instance and a set of routes
// - Populates the vector of variables and the vectors of constraints
// @param instance : the instance of the problem
// @param routes : the vector of routes
// @param route_vars : the vector of variables associated with the routes
// @param intervention_ctrs : the vector of constraints associated with the interventions
// @param vehicle_ctrs : the vector of constraints associated with the vehicles
// @param use_maximisation_formulation : whether to use the maximisation version of the master problem
// @param use_duration_only : whether to use the duration only version of the master problem
GRBModel create_model(
    const Instance& instance,
    const std::vector<Route>& routes,
    std::vector<GRBVar>& route_vars,
    std::vector<GRBVar>& postpone_vars,
    std::vector<GRBConstr>& intervention_ctrs,
    std::vector<GRBConstr>& vehicle_ctrs,
    bool use_maximisation_formulation = false,
    bool use_duration_only = false
);

// Adds a route to the model
// - Create the new variable and add it to the model & route_vars
// - Update the intervention constraints
// - Update the vehicle constraints
// - Update the objective function depending on the formulation used
void add_route(
    GRBModel& model,
    const Route& route,
    const Instance& instance,
    std::vector<GRBVar>& route_vars,
    std::vector<GRBConstr>& intervention_ctrs,
    std::vector<GRBConstr>& vehicle_ctrs,
    bool use_maximisation_formulation = false,
    bool use_duration_only = false
);


// Solve the current version of the model
// Returns a status code
int solve_model(GRBModel& model, double time_limit = -1.);

// Extract the current solution as MasterSolution object
MasterSolution extract_solution(
    const GRBModel& model,
    const std::vector<GRBVar>& route_vars,
    const std::vector<GRBConstr>& intervention_ctrs,
    const std::vector<GRBConstr>& vehicle_ctrs
);


// Set the variables to integer values
void set_integer_variables(
    GRBModel& model,
    std::vector<GRBVar>& route_vars,
    std::vector<GRBVar>& postpone_vars
);


// Set the variables to continuous values
void set_continuous_variables(
    GRBModel& model,
    std::vector<GRBVar>& route_vars,
    std::vector<GRBVar>& postpone_vars
);

// Extracts an integer solution from the model
IntegerSolution extract_integer_solution(
    const GRBModel& model,
    const std::vector<GRBVar>& route_vars
);


// Solve the model at an intermediate step, return the integer solution, and reset the variables to continuous
IntegerSolution solve_intermediary_integer_model(
    GRBModel& model,
    std::vector<GRBVar>& route_vars,
    std::vector<GRBVar>& postpone_vars,
    double time_limit = -1.
);

// Convert a list of objective values in the maximisation formulation to the minimisation formulation or vice versa
std::vector<double> convert_min_max_objective(const std::vector<double>& objectives, const Instance& instance);


// Compute the objective value of an integer solution
double compute_integer_objective(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance, bool use_min = true);





