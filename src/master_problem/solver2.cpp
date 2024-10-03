#include "solver2.h"


GRBModel create_model(
    const Instance& instance,
    const std::vector<Route>& routes,
    std::vector<GRBVar>& route_vars,
    std::vector<GRBVar>& postpone_vars,
    std::vector<GRBConstr>& intervention_ctrs,
    std::vector<GRBConstr>& vehicle_ctrs
) { 
    // Formulate and solve model
    // Next step is creating the master problem
    GRBEnv env = GRBEnv(true);
    env.set(GRB_IntParam_OutputFlag, 0);
    //env.set(GRB_DoubleParam_TimeLimit, time_limit);
    // Start the environment
    env.start();
    // Create the master problem model
    GRBModel master = GRBModel(env);
    // Create the variables - all inactive at first, upper bound is 0
    for(int r = 0; r < routes.size(); r++){
        route_vars.push_back(master.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS));
    }
    // Is the intervention postponed / outsourced ?
    for (int i = 0; i < instance.number_interventions; i++){
        postpone_vars.push_back(master.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS));
    }
    // Create the intervention constraints (each intervention is visited or postponed exactly once)
    for (int i = 0; i < instance.number_interventions; i++){
        GRBLinExpr expr = postpone_vars[i];
        for (int r = 0; r < routes.size(); r++){
            expr += routes[r].is_in_route[i] * route_vars[r];
        }
        intervention_ctrs.push_back(master.addConstr(expr >= 1));
    }
    // And the vehicle constraints (each vehicle is used at most once)
    for (int v = 0; v < instance.vehicles.size(); v++){
        GRBLinExpr expr = 0;
        for (int r = 0; r < routes.size(); r++){
            if (routes[r].vehicle_id == v){
                expr += route_vars[r];
            }
        }
        vehicle_ctrs.push_back(master.addConstr(expr <= 1));
    }

    // Finally, we set the objective function
    GRBLinExpr obj = 0;
    for (int i = 0; i < instance.number_interventions; i++){
        obj += instance.nodes[i].duration * instance.M * postpone_vars[i];
    }
    for (int r = 0; r < routes.size(); r++){
        obj += routes[r].total_cost * route_vars[r];
    }

    master.setObjective(obj, GRB_MINIMIZE);

    return master;
}


void add_route(
    GRBModel& model,
    const Route& route,
    const Instance& instance,
    std::vector<GRBVar>& route_vars,
    std::vector<GRBConstr>& intervention_ctrs,
    std::vector<GRBConstr>& vehicle_ctrs
) {
    // Create the new variable and add it to the model & route_vars
    route_vars.push_back(model.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS));
    // Update the intervention constraints
    for (int i = 0; i < intervention_ctrs.size(); i++){
        model.chgCoeff(intervention_ctrs[i], route_vars.back(), route.is_in_route[i]);
    }
    // Update the vehicle constraints
    model.chgCoeff(vehicle_ctrs[route.vehicle_id], route_vars.back(), 1);

    // Update the objective function
    route_vars.back().set(GRB_DoubleAttr_Obj, route.total_cost);
}


int solve_model(GRBModel& model) {
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