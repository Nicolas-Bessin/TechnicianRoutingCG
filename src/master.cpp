#include <gurobi_c++.h>
#include "master.h"

using namespace std;

MasterSolution cg_solver(Instance instance, double time_limit){
    // Very first step is to create a "dumb" empty route
    vector<Route*> routes;
    Route* empty_route = new Route(0, instance.nodes.size());
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

        // Get the objective value
        double objective_value = master.get(GRB_DoubleAttr_ObjVal);
        
        // Get the coefficients of the variables
        vector<double> coefficients;
        for (int i = 0; i < variables.size(); i++){
            coefficients.push_back(variables[i].get(GRB_DoubleAttr_X));
        }

        // Get the duals of the constraints
        vector<double> alphas;
        for (int i = 0; i < intervention_constraints.size(); i++){
            alphas.push_back(intervention_constraints[i].get(GRB_DoubleAttr_Pi));
        }
        vector<double> betas;
        for (int i = 0; i < vehicle_constraints.size(); i++){
            betas.push_back(vehicle_constraints[i].get(GRB_DoubleAttr_Pi));
        }

        // Print the duals
        cout << "Duals of the intervention constraints: ";
        for (int i = 0; i < alphas.size(); i++){
            cout << alphas[i] << " ";
        }
        cout << endl;
        cout << "Duals of the vehicle constraints: ";
        for (int i = 0; i < betas.size(); i++){
            cout << betas[i] << " ";
        }
        cout << endl;

        // Return the duals by building a solution
        return MasterSolution(coefficients, alphas, betas, objective_value);



    } catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch(...) {
        cout << "Exception during optimization" << endl;
    }

    
    return MasterSolution();
}