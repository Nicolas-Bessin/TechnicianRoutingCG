#include "subproblem.h"

#include "instance/constants.h"
#include "pricing_problem/time_window_lunch.h"

#include "../../pathwyse/core/solver.h"

#include "pulse/pulse.h"
#include "pulse/pulse_grouped.h"
#include "pulse/pulse_multithreaded.h"
#include "pulse/pulse_grouped_multithreaded.h"

#include "data_analysis/analysis.h"

#include <map>
#include <iostream>
#include <memory>
#include <chrono>


using std::cout, std::endl;
using std::vector, std::list, std::string;
using std::unique_ptr;


// Creates a pricing instance for a given vehicle
// Adds the constraint of capacities and time windows
// Does not initialize the objective function
// The objective function depends on the RMP formulation
unique_ptr<Problem> create_pricing_instance(
    const Instance& instance, 
    const Vehicle& vehicle,
    bool use_cyclic_pricing
    ) {
    int n_interventions_v = vehicle.interventions.size();
    int origin = n_interventions_v;
    int destination = origin + 1;

    Problem* problem = new Problem(
        std::to_string(vehicle.id),
        n_interventions_v + 2,
        origin,
        destination,
        0,
        use_cyclic_pricing,
        false,
        true
    );
    problem->initProblem();

    // First step is to define the underlying graph while taking into account the forbidden and required edges
    for (int i = 0; i < n_interventions_v ; i++) {
        if (is_edge_feasible(vehicle.depot, vehicle.interventions[i], instance)) {
            problem->setNetworkArc(origin, i);
        }
        // First, we add the arcs that go to other interventions
        for (int j = 0; j < n_interventions_v; j++) {
            if (i == j) continue;
            //Check if the arc is feasible
            if (is_edge_feasible(vehicle.interventions[i], vehicle.interventions[j], instance)) {
                problem->setNetworkArc(i, j);
            }
        }
        // Then, we add the arc that goes to the destination
        problem->setNetworkArc(i, destination);
    }

    // Initialize the objective function
    DefaultCost* objective = new DefaultCost();
    objective->initData(false, n_interventions_v + 2);
    // Set this resource as the objective function, unitialized (initialization happens in the set_pricing_instance_costs function)
    problem->setObjective(objective);

    // Create a vector of resources for the problem - resources are identical across all formulations
    vector<Resource<int>*> resources;  
    int nb_capacities = instance.capacities_labels.size();
    for (const string & label : instance.capacities_labels) {
        Capacity* capacity = new Capacity();
        capacity->initData(false, n_interventions_v + 2);
        capacity->setName(label);
        capacity->setUB(vehicle.capacities.at(label) + 1);
        for (int i = 0; i < n_interventions_v; i++) {
            const Node& intervention = instance.nodes[vehicle.interventions[i]];
            int consumption = intervention.quantities.at(label);
            capacity->setNodeCost(i, consumption);
        }
        resources.push_back(capacity);
    }

    // Add the time window ressource
    CustomTimeWindow* time_window = new CustomTimeWindow(n_interventions_v + 2);
    time_window->initData(false, n_interventions_v + 2);
    time_window->setName("Time Window + Lunch");
    // The time windows on the warehouse :
    time_window->setNodeBound(n_interventions_v + 2, origin, 0, END_DAY);
    time_window->setNodeBound(n_interventions_v + 2, destination, 0, END_DAY);
    for (int i = 0; i < n_interventions_v; i++) {
        int true_i = vehicle.interventions[i];
        const Node& intervention_i = instance.nodes[true_i];
        int start_window = intervention_i.start_window;
        int end_window = intervention_i.end_window - intervention_i.duration;
        time_window->setNodeBound(n_interventions_v + 2, i, start_window, end_window);
        time_window->setNodeCost(i, intervention_i.duration);
        time_window->setLunchConstraint(i, intervention_i.is_ambiguous);
        // Also set the time consumption on the arcs between the interventions
        for (int j = 0; j < n_interventions_v; j++) {
            if (i == j) continue;
            int true_j = vehicle.interventions[j];
            int travel_time = instance.time_matrix[true_i][true_j];
            time_window->setArcCost(i, j, travel_time);
        }
        // Arcs to/from the warehouse
        int travel_time_out = instance.time_matrix[vehicle.depot][true_i];
        int travel_time_in = instance.time_matrix[true_i][vehicle.depot];
        time_window->setArcCost(origin, i, travel_time_out);
        time_window->setArcCost(i, destination, travel_time_in);
    }
    // Add the time window ressource to the vector of resources
    time_window->init(origin, destination);
    resources.push_back(time_window);

    // Set the resources of the problem
    problem->setResources(resources);
    return unique_ptr<Problem>(problem);
}



void set_pricing_instance_costs(
    unique_ptr<Problem> & pricing_problem, 
    const DualSolution& dual_solution, 
    const Instance& instance, 
    const Vehicle& vehicle,
    SolverMode solver_objective_mode = SolverMode::BIG_M_FORMULATION_OUTSOURCE
    ) {
    int n_interventions_v = vehicle.interventions.size();
    auto objective = dynamic_cast<DefaultCost*>(pricing_problem->getObj());
    int origin = pricing_problem->getOrigin();
    int destination = pricing_problem->getDestination();
    // Set the costs of the objective function, depending on the formulation
    if (solver_objective_mode == SolverMode::DURATION_ONLY){
        for (int i = 0; i < n_interventions_v; i++) {
            int true_i = vehicle.interventions[i];
            objective->setNodeCost(i, -dual_solution.alphas[true_i]);
        }
        objective->setNodeCost(origin, - dual_solution.betas[vehicle.id]);
        
    } else if(solver_objective_mode == SolverMode::SOLUTION_MINIMISATION || solver_objective_mode == SolverMode::BIG_M_FORMULATION_OUTSOURCE){
        for (int i = 0; i < n_interventions_v; i++) {
            int true_i = vehicle.interventions[i];
            objective->setNodeCost(i, - dual_solution.alphas[true_i]);
            for (int j = 0; j < n_interventions_v; j++) {
                if (i == j) continue;
                int true_j = vehicle.interventions[j];
                // Get the distance between the two interventions
                int distance = instance.distance_matrix[true_i][true_j];
                double arc_cost = instance.cost_per_km * distance;
                objective->setArcCost(i, j, arc_cost);
            }
            // Arcs to / from the warehouse
            int distance_out = instance.distance_matrix[vehicle.depot][true_i];
            objective->setArcCost(origin, i, instance.cost_per_km * distance_out);
            int distance_in = instance.distance_matrix[true_i][vehicle.depot];
            objective->setArcCost(i, destination, instance.cost_per_km * distance_in);
        }
        objective->setNodeCost(origin, - dual_solution.betas[vehicle.id] + vehicle.cost);

    } else if (solver_objective_mode == SolverMode::BIG_M_FORMULATION_COVERAGE){
        for (int i = 0; i < n_interventions_v; i++) {
            int true_i = vehicle.interventions[i];
            const Node& intervention_i = instance.nodes[true_i];
            objective->setNodeCost(i, dual_solution.alphas[true_i] - intervention_i.duration * instance.M);
            // Arc costs
            for (int j = 0; j < n_interventions_v; j++) {
                if (i == j) continue;
                int true_j = vehicle.interventions[j];
                // Get the distance between the two interventions
                int distance = instance.distance_matrix[true_i][true_j];
                double arc_cost = instance.cost_per_km * distance;
                objective->setArcCost(i, j, arc_cost);
            }
            // Arcs to / from the warehouse
            int distance_out = instance.distance_matrix[vehicle.depot][true_i];
            objective->setArcCost(origin, i, instance.cost_per_km * distance_out);
            int distance_in = instance.distance_matrix[true_i][vehicle.depot];
            objective->setArcCost(i, destination, instance.cost_per_km * distance_in);
        }
        objective->setNodeCost(origin, dual_solution.betas[vehicle.id] + vehicle.cost);
    }
    return;
}


Route solve_pricing_problem(
    const Instance &instance, 
    const Vehicle &vehicle,
    const DualSolution &dual_solution,
    SolverMode solver_objective_mode,
    bool use_cyclic_pricing,
    int n_res_dom
    ) {
    // Create the pricing problem
    unique_ptr<Problem> pricing_problem = create_pricing_instance(instance, vehicle, use_cyclic_pricing);
    set_pricing_instance_costs(pricing_problem, dual_solution, instance, vehicle, solver_objective_mode);
    // Solve the pricing problem
    Solver solver = Solver();
    solver.setCustomProblem(*pricing_problem, true);
    solver.setupAlgorithms();
    // Set the dominance test
    if(n_res_dom == -1) {
        n_res_dom = pricing_problem->getNumRes();
    }
    dynamic_cast<PWDefault*>(solver.getMainAlgorithm())->setNResourceDomLM(n_res_dom);
    solver.solve();
    // If the problem is indeterminate (time limit reached, we return an empty vector)
    if (solver.getProblem()->getStatus() == PROBLEM_INDETERMINATE) {
        cout << "Time limit reached for vehicle " << vehicle.id << endl;
        return EmptyRoute(instance.nodes.size());
    }
    // Else, if the problem is infeasible, we return an empty vector
    if (solver.getProblem()->getStatus() == PROBLEM_INFEASIBLE) {
        //cout << "Problem is infeasible for vehicle " << vehicle.id << endl;
        return EmptyRoute(instance.nodes.size());
    }
    Path path = solver.getBestSolution();
    // Get the sequence as a vector of integers
    double reduced_cost;
    if (solver_objective_mode == SolverMode::BIG_M_FORMULATION_COVERAGE) {
        reduced_cost = - path.getObjective();
    } else {
        reduced_cost = path.getObjective();
    }
    list<int> tour_list = path.getTour();
    vector<int> tour(tour_list.begin(), tour_list.end());
    // Check that we found an elementary path
    if (!path.isElementary()) {
        cout << "Vehicle " << vehicle.id << " : Path is not elementary" << endl;
    }
    
    return convert_sequence_to_route(reduced_cost, tour, instance, vehicle);
}


std::vector<Route> solve_pricing_problem_pulse(
    const Instance &instance, 
    const Vehicle &vehicle,
    const DualSolution &dual_solution,
    SolverMode solver_objective_mode,
    int delta,
    int pool_size,
    bool verbose
    ) {
    using namespace std::chrono;
    // Create the pricing problem
    unique_ptr<Problem> pricing_problem = create_pricing_instance(instance, vehicle, true);
    set_pricing_instance_costs(pricing_problem, dual_solution, instance, vehicle, solver_objective_mode);
    // Create the pulse algorithm
    PulseAlgorithm pulse_algorithm = PulseAlgorithm(pricing_problem.get(), delta, pool_size);
    // Bounding phase
    auto start = steady_clock::now();
    pulse_algorithm.bound();
    auto end = steady_clock::now();
    int duration_bound = duration_cast<milliseconds>(end - start).count();

    // Solve the pricing problem
    auto start_pricing = steady_clock::now();
    int error = pulse_algorithm.solve();

    if (error != 0) {
        cout << "Error in solving the pulse algorithm" << endl;
        return {};
    }

    // Transform the partial pathes from the solution pool into Route objects
    std::vector<Route> new_routes;
        
    for (const auto& [rc, path] : pulse_algorithm.get_solution_pool()) {
        double reduced_cost;
        if (solver_objective_mode == SolverMode::BIG_M_FORMULATION_COVERAGE) {
            reduced_cost = - rc;
        } else {
            reduced_cost = rc;
        }
        new_routes.push_back(convert_sequence_to_route(reduced_cost, path.sequence, instance, vehicle));
    }
    auto end_pricing = steady_clock::now();
    int duration_pricing = duration_cast<milliseconds>(end_pricing - start_pricing).count();
    if (verbose) {
        cout << "V" << vehicle.id << " : Bound in " << duration_bound << " ms, pricing in " << duration_pricing << " ms" << endl;
    }

    return new_routes;
}


Vehicle create_virtual_vehicle(const Instance & instance, const std::vector<int> & vehicle_indexes) {
    using std::set, std::map, std::vector;
    // At firts, we create a virtual vehicle that contains all the interventions
    if (vehicle_indexes.size() == 0) {
        std::cerr << "No vehicle indexes provided" << endl;
        return {};
    }
    set<int> all_interventions;
    set<int> depots;
    map<std::string, int> all_capacities = instance.vehicles[vehicle_indexes[0]].capacities;
    for (int v : vehicle_indexes) {
        depots.insert(instance.vehicles[v].depot);
        all_interventions.insert(instance.vehicles[v].interventions.begin(), instance.vehicles[v].interventions.end());
        for (const auto& [label, capacity] : instance.vehicles[v].capacities) {
            all_capacities[label] = std::max(all_capacities[label], capacity);
        }
    }
    if (depots.size() != 1) {
        std::cerr << "All vehicles need to have the same depot" << endl;
        return {};
    }
    vector<int> all_interventions_v;
    map<int, int> reverse_interventions;
    for (int i : all_interventions) {
        all_interventions_v.push_back(i);
        reverse_interventions[i] = all_interventions_v.size() - 1;
    }
    Vehicle virtual_vehicle = Vehicle{
        -1, // Fake id
        {}, // Technicians are not needed
        {}, // Skills are not needed
        all_interventions_v,
        reverse_interventions,
        *depots.begin(),
        all_capacities,
        0 // Fake cost
    };
    return virtual_vehicle;
}


std::vector<int> get_available_interventions(const Vehicle & true_vehicle, const Vehicle & virtual_vehicle) {
    std::vector<int> available_interventions_v(virtual_vehicle.interventions.size() + 2, 0);
    for (int i : true_vehicle.interventions) {
        available_interventions_v[virtual_vehicle.reverse_interventions.at(i)] = 1;
    }
    // Last two nodes are the depot and the destination, always available
    available_interventions_v[available_interventions_v.size() - 2] = 1;
    available_interventions_v[available_interventions_v.size() - 1] = 1;
    return available_interventions_v;
}


std::vector<Route> solve_pricing_problem_pulse_grouped(
    const Instance &instance, 
    const std::vector<int> & vehicle_indexes,
    const DualSolution &dual_solution,
    SolverMode solver_objective_mode,
    int delta,
    int pool_size,
    bool verbose
) {
    using std::vector;
    using namespace std::chrono;

    auto virtual_vehicle = create_virtual_vehicle(instance, vehicle_indexes);
    
    int n_interventions_v = virtual_vehicle.interventions.size();

    // Create the pricing problem
    unique_ptr<Problem> pricing_problem = create_pricing_instance(instance, virtual_vehicle, true);
    set_pricing_instance_costs(pricing_problem, dual_solution, instance, virtual_vehicle, solver_objective_mode);

    // Create the pulse algorithm
    PulseAlgorithmWithSubsets pulse_algorithm = PulseAlgorithmWithSubsets(pricing_problem.get(), delta, pool_size);
    // Set all the interventions as available
    pulse_algorithm.reset();
    // Proceed with the bounding phase
    auto start = steady_clock::now();
    pulse_algorithm.bound();
    auto end = steady_clock::now();
    int duration_bound = duration_cast<milliseconds>(end - start).count();


    // For every vehicle, define the available interventions and solve the pricing problem
    auto start_pricing = steady_clock::now();
    vector<Route> new_routes;
    for (int v : vehicle_indexes) {
        // Get the vehicle
        const Vehicle& vehicle = instance.vehicles[v];
        auto available_interventions = get_available_interventions(vehicle, virtual_vehicle);
        // Solve the pricing problem
        int error = pulse_algorithm.solve(available_interventions);
        if (error != 0) {
            cout << "Error in solving the pulse algorithm for vehicle " << v << endl;
            continue;
        }
        // Transform the partial pathes from the solution pool into Route objects
        for (const auto& [rc, path] : pulse_algorithm.get_solution_pool()) {
            double reduced_cost;
            if (solver_objective_mode == SolverMode::BIG_M_FORMULATION_COVERAGE) {
                reduced_cost = - rc;
            } else {
                reduced_cost = rc;
            }
            Route route = convert_sequence_to_route(reduced_cost, path.sequence, instance, virtual_vehicle);
            // Update the route with the real vehicle id & add the cost of the vehicle
            route.vehicle_id = v;
            route.total_cost += vehicle.cost;
            new_routes.push_back(route);
        }
    }
    auto end_pricing = steady_clock::now();
    int duration_pricing = duration_cast<milliseconds>(end_pricing - start_pricing).count();

    if (verbose) {
        cout << "Group [";
        for (int v : vehicle_indexes) {
            cout << "v" << v << ", ";
        }
        cout << "] : Bound in " << duration_bound << " ms, pricing in " << duration_pricing << " ms" << endl;
    }

    return new_routes;
}