#include "RMP_solver.h"

#include "gurobi_c++.h"


using std::vector;
using std::cout, std::endl;
using std::string;

MasterSolution relaxed_RMP(const Instance& instance, const vector<Route>& routes, const BPNode& node){
    try {
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
        vector<GRBVar> variables;
        for(int r = 0; r < routes.size(); r++){
            variables.push_back(master.addVar(0.0, 0.0, 0.0, GRB_CONTINUOUS));
        }
        // Only activate the variables that are active in the node
        for (int r : node.active_routes){
            variables[r].set(GRB_DoubleAttr_UB, GRB_INFINITY);
        }
        // Create the intervention constraints (each intervention is visited at most once)
        vector<GRBConstr> intervention_constraints;
        for (int i = 0; i < instance.number_interventions; i++){
            GRBLinExpr expr = 0;
            for (int r = 0; r < routes.size(); r++){
                expr += routes[r].is_in_route[i] * variables[r];
            }
            intervention_constraints.push_back(master.addConstr(expr <= 1));
        }
        // And the vehicle constraints (each vehicle is used at most once)
        vector<GRBConstr> vehicle_constraints;
        for (int v = 0; v < instance.vehicles.size(); v++){
            GRBLinExpr expr = 0;
            for (int r = 0; r < routes.size(); r++){
                if (routes[r].vehicle_id == v){
                    expr += variables[r];
                }
            }
            vehicle_constraints.push_back(master.addConstr(expr <= 1));
        }

        // Finally, we set the objective function
        GRBLinExpr obj = 0;
        for (int r = 0; r < routes.size(); r++){
            double coef = instance.M * routes[r].total_duration - routes[r].total_cost;
            obj += coef * variables[r];
        }
        master.setObjective(obj, GRB_MAXIMIZE);

        // Solve the master problem
        master.optimize();

        // Get the objective value
        double objective_value = master.get(GRB_DoubleAttr_ObjVal);
        
        // Get the coefficients of the variables
        vector<double> coefficients;
        for (const GRBVar& var : variables){
            coefficients.push_back(var.get(GRB_DoubleAttr_X));
        }

        // Get the duals of the constraints
        vector<double> alphas;
        for (const GRBConstr& constraint : intervention_constraints){
            alphas.push_back(constraint.get(GRB_DoubleAttr_Pi));
        }
        vector<double> betas;
        for (const GRBConstr& constraint : vehicle_constraints){
            betas.push_back(constraint.get(GRB_DoubleAttr_Pi));
        }

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


IntegerSolution integer_RMP(const Instance& instance, const vector<Route>& routes, const BPNode& node){
    try {
        // Formulate and solve model
        // Next step is creating the master problem
        GRBEnv env = GRBEnv(true);
        env.set(GRB_IntParam_OutputFlag, 0);
        // Start the environment
        env.start();
        // Create the master problem model
        GRBModel master = GRBModel(env);
        // Create the variables, all inactive at first, upper bound is 0
        vector<GRBVar> variables;
        for(int r = 0; r < routes.size(); r++){
            variables.push_back(master.addVar(0, 0, 0, GRB_INTEGER));
        }
        // Only activate the variables that are active in the node
        for (int r : node.active_routes){
            variables[r].set(GRB_DoubleAttr_UB, GRB_INFINITY);
        }
        // Create the intervention constraints (each intervention is visited at most once)
        vector<GRBConstr> intervention_constraints;
        for (int i = 0; i < instance.number_interventions; i++){
            GRBLinExpr expr = 0;
            for (int r = 0; r < routes.size(); r++){
                expr += routes[r].is_in_route[i] * variables[r];
            }
            intervention_constraints.push_back(master.addConstr(expr <= 1));
        }
        // And the vehicle constraints (each vehicle is used at most once)
        vector<GRBConstr> vehicle_constraints;
        for (int v = 0; v < instance.vehicles.size(); v++){
            GRBLinExpr expr = 0;
            for (int r = 0; r < routes.size(); r++){
                if (routes[r].vehicle_id == v){
                    expr += variables[r];
                }
            }
            vehicle_constraints.push_back(master.addConstr(expr <= 1));
        }

        // Test : impose using more than 15 vehicles
        GRBLinExpr expr = 0;
        for (int r = 0; r < routes.size(); r++){
            expr += variables[r];
        }
        //master.addConstr(expr >= 14);

        // Test : impose covering more than 75 interventions
        GRBLinExpr expr2 = 0;
        for (int i = 0; i < instance.number_interventions; i++){
            for (int r = 0; r < routes.size(); r++){
                expr2 += routes[r].is_in_route[i] * variables[r];
            }
        }
        //master.addConstr(expr2 >= 75);

        // Finally, we set the objective function
        GRBLinExpr obj = 0;
        for (int r = 0; r < routes.size(); r++){
            double coef = instance.M * routes[r].total_duration - routes[r].total_cost;
            obj += coef * variables[r];
        }
        master.setObjective(obj, GRB_MAXIMIZE);

        // Solve the master problem
        master.optimize();

        // Get the objective value
        double objective_value = master.get(GRB_DoubleAttr_ObjVal);
        // Get the coefficients of the variables
        vector<int> coefficients;
        for (int i = 0; i < variables.size(); i++){
            coefficients.push_back(variables[i].get(GRB_DoubleAttr_X));
        }

        // Return the solution
        return IntegerSolution(coefficients, objective_value);

    } catch(GRBException e) {
        cout << "Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage() << endl;
    } catch(...) {
        cout << "Exception during optimization" << endl;
    }
    return IntegerSolution();
}
    