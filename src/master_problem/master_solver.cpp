#include "master_solver.h"

#include "data_analysis/analysis.h"

#include <assert.h>


GRBModel create_model(
    const Instance& instance,
    const std::vector<Route>& routes,
    std::vector<GRBVar>& route_vars,
    std::vector<GRBVar>& postpone_vars,
    std::vector<GRBConstr>& intervention_ctrs,
    std::vector<GRBConstr>& vehicle_ctrs,
    SolverMode solver_objective_mode
) { 
    GRBEnv env = GRBEnv(true);
    env.set(GRB_IntParam_OutputFlag, 0);
    env.start();
    GRBModel master = GRBModel(env);

    // ----------------- Variables -----------------
    for(int r = 0; r < routes.size(); r++){
        route_vars.push_back(master.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS));
    }
    if (solver_objective_mode != SolverMode::BIG_M_FORMULATION_COVERAGE){
        for (int i = 0; i < instance.number_interventions; i++){
            postpone_vars.push_back(master.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS));
        }
    }

    // ----------------- Constraints -----------------
    // The vehicle vehicle constraints (each vehicle is used at most once)
    for (int v = 0; v < instance.vehicles.size(); v++){
        GRBLinExpr expr = 0;
        for (int r = 0; r < routes.size(); r++){
            if (routes[r].vehicle_id == v){
                expr += route_vars[r];
            }
        }
        vehicle_ctrs.push_back(master.addConstr(expr <= 1));
    }
    // Intervention constraints depends on the formulation
    if (solver_objective_mode == SolverMode::BIG_M_FORMULATION_COVERAGE){
        for (int i = 0; i < instance.number_interventions; i++){
            GRBLinExpr expr = 0;
            for (int r = 0; r < routes.size(); r++){
                expr += routes[r].is_in_route[i] * route_vars[r];
            }
            intervention_ctrs.push_back(master.addConstr(expr <= 1));
        }
    } else {
        for (int i = 0; i < instance.number_interventions; i++){
            GRBLinExpr expr = postpone_vars[i];
            for (int r = 0; r < routes.size(); r++){
                expr += routes[r].is_in_route[i] * route_vars[r];
            }
            intervention_ctrs.push_back(master.addConstr(expr >= 1));
        }
    }

    // ----------------- Objective -----------------
    if (solver_objective_mode == SolverMode::DURATION_ONLY){
        GRBLinExpr objective = 0;
        for (int i = 0; i < instance.number_interventions; i++){
            objective += instance.nodes[i].duration * postpone_vars[i];
        }
        master.setObjective(objective, GRB_MINIMIZE);

    } else if (solver_objective_mode == SolverMode::SOLUTION_MINIMISATION){
        GRBLinExpr objective = 0;
        for (int r = 0; r < routes.size(); r++){
            objective += routes[r].total_cost * route_vars[r];
        }
        master.setObjective(objective, GRB_MINIMIZE);

    } else if (solver_objective_mode == SolverMode::BIG_M_FORMULATION_OUTSOURCE){
        GRBLinExpr objective = 0;
        for (int i = 0; i < instance.number_interventions; i++){
            objective += instance.M * instance.nodes[i].duration * postpone_vars[i];
        }
        for (int r = 0; r < routes.size(); r++){
            objective += routes[r].total_cost * route_vars[r];
        }
        master.setObjective(objective, GRB_MINIMIZE);

    } else if (solver_objective_mode == SolverMode::BIG_M_FORMULATION_COVERAGE){
        GRBLinExpr objective = 0;
        for (int r = 0; r < routes.size(); r++){
            objective += (instance.M * routes[r].total_duration - routes[r].total_cost) * route_vars[r];
        }
        master.setObjective(objective, GRB_MAXIMIZE);
    }

    return master;
}


void add_route(
    GRBModel& model,
    const Route& route,
    const Instance& instance,
    std::vector<GRBVar>& route_vars,
    std::vector<GRBConstr>& intervention_ctrs,
    std::vector<GRBConstr>& vehicle_ctrs,
    SolverMode solver_objective_mode
) {
    // Create the new variable and add it to the model & route_vars
    route_vars.push_back(model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS));
    // Update the intervention constraints - same across all formulations
    for (int i = 0; i < intervention_ctrs.size(); i++){
        model.chgCoeff(intervention_ctrs[i], route_vars.back(), route.is_in_route[i]);
    }
    // Update the vehicle constraints - same across all formulations
    model.chgCoeff(vehicle_ctrs[route.vehicle_id], route_vars.back(), 1);
    // Update the objective coefficients depending on the formulation
    if (solver_objective_mode == SolverMode::SOLUTION_MINIMISATION) {
        route_vars.back().set(GRB_DoubleAttr_Obj, route.total_cost);
    } else if (solver_objective_mode == SolverMode::BIG_M_FORMULATION_OUTSOURCE) {
        route_vars.back().set(GRB_DoubleAttr_Obj, route.total_cost);
    } else if (solver_objective_mode == SolverMode::BIG_M_FORMULATION_COVERAGE) {
        route_vars.back().set(GRB_DoubleAttr_Obj, instance.M * route.total_duration - route.total_cost);
    } 

}

int solve_model(GRBModel& model, double time_limit) {
    if (time_limit > 0){
        model.set(GRB_DoubleParam_TimeLimit, time_limit);
    }
    // Solve the current version of the model
    model.optimize();
    return model.get(GRB_IntAttr_Status);
}


MasterSolution extract_solution(
    const GRBModel& model,
    const std::vector<GRBVar>& route_vars,
    const std::vector<GRBConstr>& intervention_ctrs,
    const std::vector<GRBConstr>& vehicle_ctrs
) {
    using std::vector;
    // Get the objective value
    double objective_value = model.get(GRB_DoubleAttr_ObjVal);
    
    // Get the coefficients of the variables
    vector<double> coefficients;
    for (const GRBVar& var : route_vars){
        coefficients.push_back(var.get(GRB_DoubleAttr_X));
    }

    // Get the duals of the constraints
    vector<double> alphas;
    for (const GRBConstr& constraint : intervention_ctrs){
        alphas.push_back(constraint.get(GRB_DoubleAttr_Pi));
    }
    vector<double> betas;
    for (const GRBConstr& constraint : vehicle_ctrs){
        betas.push_back(constraint.get(GRB_DoubleAttr_Pi));
    }

    return MasterSolution{coefficients, alphas, betas, objective_value};

}


void set_integer_variables(
    GRBModel& model,
    std::vector<GRBVar>& route_vars,
    std::vector<GRBVar>& postpone_vars
) {
    for (GRBVar& var : route_vars){
        var.set(GRB_CharAttr_VType, GRB_BINARY);
    }
    for (GRBVar& var : postpone_vars){
        var.set(GRB_CharAttr_VType, GRB_BINARY);
    }
}


void set_continuous_variables(
    GRBModel& model,
    std::vector<GRBVar>& route_vars,
    std::vector<GRBVar>& postpone_vars
) {
    for (GRBVar& var : route_vars){
        var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
    }
    for (GRBVar& var : postpone_vars){
        var.set(GRB_CharAttr_VType, GRB_CONTINUOUS);
    }
}


IntegerSolution extract_integer_solution(
    const GRBModel& model,
    const std::vector<GRBVar>& route_vars
) {
    using std::vector;
    // Get the objective value
    double objective_value = model.get(GRB_DoubleAttr_ObjVal);
    
    // Get the coefficients of the variables
    vector<int> coefficients = vector<int>(route_vars.size());
    for (int i = 0; i < route_vars.size(); i++){
        coefficients[i] = route_vars[i].get(GRB_DoubleAttr_X);
    }

    return IntegerSolution{coefficients, objective_value};
}


IntegerSolution solve_intermediary_integer_model(
    GRBModel& model,
    std::vector<GRBVar>& route_vars,
    std::vector<GRBVar>& postpone_vars,
    double time_limit
) {
    set_integer_variables(model, route_vars, postpone_vars);
    int status = solve_model(model, time_limit);
    auto solution = extract_integer_solution(model, route_vars);
    // Return the variables to continuous values
    set_continuous_variables(model, route_vars, postpone_vars);
    return solution;
}


std::vector<double> convert_min_max_objective(const std::vector<double>& objectives, const Instance& instance){
    // Compute the constant part of the objective
    double constant_part = 0;
    for (int i = 0; i < instance.number_interventions; i++){
        constant_part += instance.nodes[i].duration * instance.M;
    }

    // Compute the new objective values
    std::vector<double> new_objective;
    for (double value : objectives){
        new_objective.push_back(constant_part - value);
    };  

    return new_objective;  
}


double compute_integer_objective(const IntegerSolution& solution, const std::vector<Route>& routes, const Instance& instance, bool use_min) {
    double value = 0;
    assert (solution.coefficients.size() == routes.size());
    if (use_min){
        // First, count the total cost of the routes
        for (int r = 0; r < routes.size(); r++) {
            if (solution.coefficients[r] > 0) {
                double coef = 0;
                coef += instance.cost_per_km * count_route_kilometres(routes[r], instance);
                coef += instance.vehicles[routes[r].vehicle_id].cost;
                value += coef;
            }
        }
        // Then, count the value of the covered interventions
        auto covered = covered_interventions(solution, routes, instance);
        for (int i = 0; i < covered.size(); i++) {
            if (covered[i] == 0) {
                value += instance.nodes[i].duration * instance.M;
            }
        }
    } else {
        // First, count the total cost of the routes
        for (int r = 0; r < routes.size(); r++) {
            if (solution.coefficients[r] > 0) {
                double coef = 0;
                coef -= instance.cost_per_km * count_route_kilometres(routes[r], instance);
                coef -= instance.vehicles[routes[r].vehicle_id].cost;
                value += coef;
            }
        }
        // Then, count the value of the covered interventions
        auto covered = covered_interventions(solution, routes, instance);
        for (int i = 0; i < covered.size(); i++) {
            if (covered[i] > 0) {
                value += instance.nodes[i].duration * instance.M;
            }
        }
    }
    return value;
}

