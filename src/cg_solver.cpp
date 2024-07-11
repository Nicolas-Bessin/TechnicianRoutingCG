#include "cg_solver.h"

using namespace std;

void cg_solver(Instance instance, double time_limit){
    // Very first step is to create a "dumb" empty route
    vector<Route*> routes;
    Route* empty_route = new Route(0, 0, instance.nodes.size());
    //routes.push_back(empty_route);

    try {
        // Formulate and solve model
        // Next step is creating the master problem
        GRBEnv env = GRBEnv(true);
        env.set(GRB_IntParam_OutputFlag, 1);
        env.set(GRB_DoubleParam_TimeLimit, time_limit);
        // Start the environment
        env.start();
        // Create the master problem model
        GRBModel master = GRBModel(env);
        // Create the variables
        vector<GRBVar> variables;
        for(int i = 0; i < instance.nodes.size(); i++){
            variables.push_back(master.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS));
        }
        // Create the intervention constraints (each intervention is visited at most once)
        vector<GRBConstr> intervention_constraints;
        for (int i = 0; i < instance.nodes.size(); i++){
            GRBLinExpr expr = 0;
            for (int j = 0; j < routes.size(); j++){
                expr += (*routes[j]).is_in_route[i] * variables[j];
            }
            GRBConstr constraint = master.addConstr(expr <= 1);
            intervention_constraints.push_back(constraint);
        }
        // And the vehicle constraints (each vehicle is used at most once)
        vector<GRBConstr> vehicle_constraints;
        for (int i = 0; i < instance.vehicles.size(); i++){
            GRBLinExpr expr = 0;
            for (auto route : routes){
                expr += route->vehicle_id == i ? 1 : 0;
            }
            GRBConstr constraint = master.addConstr(expr <= 1);
            vehicle_constraints.push_back(constraint);
        }

        // Finally, we set the objective function
        GRBLinExpr obj = 0;
        for (int r = 0; r < routes.size(); r++){
            double coef = instance.M * routes[r]->total_duration - routes[r]->total_cost;
            obj += coef * variables[r];
        }
        master.setObjective(obj, GRB_MINIMIZE);

        // Solve the master problem
        master.optimize();
    } catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch(...) {
        cout << "Exception during optimization" << endl;
    }

    
    return;
}