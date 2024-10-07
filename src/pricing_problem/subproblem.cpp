#include "subproblem.h"

#include "instance/constants.h"
#include "pricing_problem/time_window_lunch.h"
#include "../../pathwyse/core/solver.h"
#include "pulse/pulse.h"
#include "data_analysis/analysis.h"

#include <map>
#include <iostream>
#include <memory>
#include <chrono>


using std::cout, std::endl;
using std::vector, std::list, std::string;
using std::unique_ptr;


unique_ptr<Problem> create_pricing_instance(
    const Instance& instance, 
    const Vehicle& vehicle,
    bool use_cyclic_pricing,
    const std::set<std::tuple<int, int, int>>& forbidden_edges,
    const std::set<std::tuple<int, int, int>>& required_edges
    ) {
    // in hopes of mitigating the issue of the costs & objctive values only being integers in pathwyse
    
    // Get the number of nodes in the problem : equal to the number of available interventions + 2
    // +1 for the "depature" warehouse and +1 for the "arrival" warehouse (even tough it is the same place)
    int n_interventions_v = vehicle.interventions.size();
    int origin = n_interventions_v;
    int destination = origin + 1;
    // Get the number of resources in the problem

    // Create a new instance of the Problem class
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
    // Initialize the problem
    problem->initProblem();

    // First step is to define the underlying graph while taking into account the forbidden and required edges
    // We do not add the arcs that are forbidden
    // And if arc (i, j) is required, we only add this particular arc
    // Begin with the arcs that go from the warehouse to the interventions
    // If there is a required edge, we only add this edge
    bool required_first_edge = false;
    for (const auto& [i_, j_, v_] : required_edges) {
        // Skip every edge not related to this vehicle
        if (v_ != vehicle.id) continue;
        if (i_ == vehicle.depot) {
            required_first_edge = true;
            int reverse_j;
            if (j_ == vehicle.depot) {
                reverse_j = destination;
            } else {
                if (!vehicle.reverse_interventions.contains(j_)) {
                    cout << "Vehicle " << vehicle.id << " can not do intervention " << j_ << endl;
                    break;
                }
                reverse_j = vehicle.reverse_interventions.at(j_);
            }
            problem->setNetworkArc(origin, reverse_j);
            break;
        }
    }
    for (int i = 0; i < n_interventions_v ; i++) {
        // If there wasn't a required edge from the depot, we add the arc from the depot to the intervention
        // Provided it is not forbidden
        if (!required_first_edge && !forbidden_edges.contains(std::make_tuple(vehicle.depot, vehicle.interventions[i], vehicle.id))) {
            problem->setNetworkArc(origin, i);
        }
        // Then, is there a required edge starting from i ?
        bool required_edge = false;
        for (const auto& [i_, j_, v_] : required_edges) {
            if (v_ != vehicle.id) continue;
            if (i_ == vehicle.interventions[i]) {
                required_edge = true;
                int reverse_j;
                if (j_ == vehicle.depot) {
                    reverse_j = destination;
                } else {
                    if (!vehicle.reverse_interventions.contains(j_)) {
                        cout << "Vehicle " << vehicle.id << " can not do intervention " << j_ << endl;
                        break;
                    }
                    reverse_j = vehicle.reverse_interventions.at(j_);
                }
                problem->setNetworkArc(i, reverse_j);
                break;
            }
        }
        if (required_edge) continue;
        // If not, we add the arcs that are not forbidden
        // First, we add the arcs that go to other interventions
        for (int j = 0; j < n_interventions_v; j++) {
            if (i == j) continue;
            // Check if the arc is feasible
            // bool feasible = is_edge_feasible(vehicle.interventions[i], vehicle.interventions[j], instance);
            // if (!feasible) continue;
            // Check if the arc is forbidden
            bool forbidden = forbidden_edges.contains(std::make_tuple(vehicle.interventions[i], vehicle.interventions[j], vehicle.id));
            if (!forbidden) problem->setNetworkArc(i, j);
        }
        // Then, we add the arc that goes to the destination
        if (!forbidden_edges.contains(std::make_tuple(vehicle.interventions[i], vehicle.depot, vehicle.id))) {
            problem->setNetworkArc(i, destination);
        }
    }

    // Initialize the objective function
    DefaultCost* objective = new DefaultCost();
    objective->initData(false, n_interventions_v + 2);

    // Set the costs of the arcs
    for (int i = 0; i < n_interventions_v; i++) {
        int true_i = vehicle.interventions[i];
        const Node& intervention_i = (instance.nodes[vehicle.interventions[i]]);
        // Set the node cost of the intervention (used when using the pricing problem as heuristics)
        // Otherwise, we overwrite this value when updating the pricing problem with dual values
        objective->setNodeCost(i, -instance.M * intervention_i.duration);
        for (int j = 0; j < n_interventions_v; j++) {
            if (i == j) continue;
            int true_j = vehicle.interventions[j];
            // Get the intervention referenced by the index j
            const Node& intervention_j = instance.nodes[vehicle.interventions[j]];
            // Get the distance between the two interventions
            int distance = instance.distance_matrix[true_i][true_j];
            double arc_cost = instance.cost_per_km * distance;
            objective->setArcCost(i, j, arc_cost);
        }
        // Outgoing arcs from the warehouse
        int distance_out = instance.distance_matrix[vehicle.depot][true_i];
        // Set the arc cost
        objective->setArcCost(origin, i, instance.cost_per_km * distance_out);
        // Incoming arcs to the warehouse
        // Distance is not the same as the outgoing distance
        int distance_in = instance.distance_matrix[true_i][vehicle.depot];
        // Here we only consider the cost of travelling, the node costs have already been set
        objective->setArcCost(i, destination, instance.cost_per_km * distance_in);
    }

    // Set this resource as the objective function
    problem->setObjective(objective);

    // Create a vector of resources for the problem
    vector<Resource<int>*> resources;  

    // We then want to add the capacity resources to the problem
    int nb_capacities = instance.capacities_labels.size();
    // For every label of capacity, we create a new capacity ressource
    for (const string & label : instance.capacities_labels) {
        // Create a new capacity ressource
        Capacity* capacity = new Capacity();
        capacity->initData(false, n_interventions_v + 2);
        // Set the name of the ressource
        capacity->setName(label);
        // Set its upper bound
        capacity->setUB(vehicle.capacities.at(label) + 1);
        // Set the node consumptions for the ressource
        for (int i = 0; i < n_interventions_v; i++) {
            // Get the intervention referenced by the index i
            const Node* intervention = &(instance.nodes[vehicle.interventions[i]]);
            // Get the capacity consumption of the intervention
            int consumption = intervention->quantities.at(label);
            // Set the capacity consumption of the intervention
            capacity->setNodeCost(i, consumption);
        }
        // Add it to the vector of resources
        resources.push_back(capacity);
    }

    // Add the time window ressource to the problem as the critical ressource
    CustomTimeWindow* time_window = new CustomTimeWindow(n_interventions_v + 2);
    time_window->initData(false, n_interventions_v + 2);
    time_window->setName("Time Window + Lunch");
    // For each node, add the UB and LB and node consumption
    for (int i = 0; i < n_interventions_v; i++) {
        int true_i = vehicle.interventions[i];
        const Node& intervention_i = instance.nodes[true_i];
        // Get the time window of the intervention
        int start_window = intervention_i.start_window;
        int end_window = intervention_i.end_window - intervention_i.duration;
        // Set the time window of the intervention
        time_window->setNodeBound(n_interventions_v + 2, i, start_window, end_window);
        // Set the node consumption of the time window
        time_window->setNodeCost(i, intervention_i.duration);
        // Sets the lunch break constraint
        time_window->setLunchConstraint(i, intervention_i.is_ambiguous);
        // Also set the time consumption on the arcs between the interventions
        for (int j = 0; j < n_interventions_v; j++) {
            if (i == j) continue;
            int true_j = vehicle.interventions[j];
            // Get the time it takes to go from intervention i to intervention j
            int travel_time = instance.time_matrix[true_i][true_j];
            // Set the time consumption on the arc
            time_window->setArcCost(i, j, travel_time);
        }

        // Outgoing arcs from the warehouse
        int travel_time_out = instance.time_matrix[vehicle.depot][true_i];
        int travel_time_in = instance.time_matrix[true_i][vehicle.depot];
        time_window->setArcCost(origin, i, travel_time_out);
        time_window->setArcCost(i, destination, travel_time_in);
    }
    // The time windows on the warehouse :
    time_window->setNodeBound(n_interventions_v + 2, origin, 0, END_DAY);
    time_window->setNodeBound(n_interventions_v + 2, destination, 0, END_DAY);
   
    // Add the time window ressource to the vector of resources
    time_window->init(origin, destination);
    resources.push_back(time_window);


    // Set the resources of the problem
    problem->setResources(resources);
    //problem->printProblem();

    return unique_ptr<Problem>(problem);
}



void update_pricing_instance(
    unique_ptr<Problem> & pricing_problem, 
    const DualSolution& dual_solution, 
    const Instance& instance, 
    const Vehicle& vehicle) {
    // Get the number of nodes in the problem : equal to the number of available interventions + 2
    int n_interventions_v = vehicle.interventions.size();
    int origin = n_interventions_v;
    int destination = origin + 1;
    // Get the objective function of the problem
    DefaultCost* objective = (DefaultCost*) pricing_problem->getObj();
    // Put in the node costs : alpha_i - M * duration_i
    for (int i = 0; i < n_interventions_v; i++) {
        int true_i = vehicle.interventions[i];
        double node_cost = - dual_solution.alphas[true_i];
        objective->setNodeCost(i, node_cost);
    }
    // Add the arcs costs where applicable from the dual values of the cuts
    for (const auto& [ijv, value] : dual_solution.upper_bound_duals) {
        int true_i = std::get<0>(ijv);
        int true_j = std::get<1>(ijv);
        int v = std::get<2>(ijv);
        if (v != vehicle.id) continue;
        int i = vehicle.reverse_interventions.at(true_i);
        int j = vehicle.reverse_interventions.at(true_j);
        double arc_cost = value + instance.cost_per_km * instance.distance_matrix[true_i][true_j];
        objective->setArcCost(i, j, arc_cost);
    }
    // Same thing with the lower bound duals
    for (const auto& [ijv, value] : dual_solution.lower_bound_duals) {
        int true_i = std::get<0>(ijv);
        int true_j = std::get<1>(ijv);
        int v = std::get<2>(ijv);
        if (v != vehicle.id) continue;
        int i = vehicle.reverse_interventions.at(true_i);
        int j = vehicle.reverse_interventions.at(true_j);
        double arc_cost = -value + instance.cost_per_km * instance.distance_matrix[true_i][true_j];
        objective->setArcCost(i, j, arc_cost);
    }
    
    // Put in the fixed costs of the vehicle
    if (vehicle.id == -1) {
        return;
    }
    double fixed_cost = - dual_solution.betas[vehicle.id] + vehicle.cost;
    objective->setNodeCost(origin, fixed_cost);
    return;
}


Route solve_problem(
    unique_ptr<Problem> & problem,
    const Instance& instance, 
    const Vehicle& vehicle,
    int n_res_dom
    ) {
    // Get the origin and destination of the problem
    int origin = problem->getOrigin();
    int destination = problem->getDestination();
    // We now want to solve the problem
    // We give the solver a different set of parameters depending on the problem
    int param_mode = DEFAULT_ACYCLIC_PARAM_MODE;
    if (problem->isGraphCyclic()) {
        param_mode = DEFAULT_CYCLE_PARAM_MODE;
    }
    Solver solver = Solver(param_mode);
    solver.setCustomProblem(*problem, true);
    solver.setupAlgorithms();
    // Set the dominance test
    if(n_res_dom == -1) {
        n_res_dom = problem->getNumRes();
    }
    PWDefault* algorithm = static_cast<PWDefault*>(solver.getMainAlgorithm());
    algorithm->setNResourceDomLM(n_res_dom);
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
    //solver.printBestSolution();
    // Get the solution we found
    Path path = solver.getBestSolution();
    //cout << "Solver found " << solver.getNumberOfSolutions() << " solutions" << endl;
    // Convert the Path object to a Route object
    // Get the sequence as a vector of integers
    double reduced_cost = path.getObjective(); // (We got the max reduced cost by minimizing the opposite of the reduced cost)
    list<int> tour_list = path.getTour();
    vector<int> tour(tour_list.begin(), tour_list.end());
    // Check that we found an elementary path
    if (!path.isElementary()) {
         cout << "Vehicle " << vehicle.id << " : Path is not elementary" << endl;
    }
    
    return convert_sequence_to_route(reduced_cost, tour, instance, vehicle);
    
}

Route solve_pricing_problem(
    const Instance &instance, 
    const Vehicle &vehicle,
    const DualSolution &dual_solution,
    bool use_cyclic_pricing,
    int n_res_dom,
    const std::set<std::tuple<int, int, int>> &forbidden_edges,
    const std::set<std::tuple<int, int, int>> &required_edges
    ) {
    // Create the pricing problem
    unique_ptr<Problem> pricing_problem = create_pricing_instance(instance, vehicle, use_cyclic_pricing, forbidden_edges, required_edges);
    // Update the pricing problem with the dual values
    update_pricing_instance(pricing_problem, dual_solution, instance, vehicle);
    // Solve the pricing problem
    return solve_problem(pricing_problem, instance, vehicle, n_res_dom);
}


std::vector<Route> solve_pricing_problem_pulse(
    const Instance &instance, 
    const Vehicle &vehicle,
    const DualSolution &dual_solution,
    int delta,
    int pool_size,
    const std::set<std::tuple<int, int, int>> &forbidden_edges,
    const std::set<std::tuple<int, int, int>> &required_edges
    ) {
    using namespace std::chrono;
    // Create the pricing problem
    unique_ptr<Problem> pricing_problem = create_pricing_instance(instance, vehicle, true, forbidden_edges, required_edges);
    // Update the pricing problem with the dual values
    update_pricing_instance(pricing_problem, dual_solution, instance, vehicle);
    // Create the pulse algorithm
    PulseAlgorithm pulse_algorithm = PulseAlgorithm(pricing_problem.get(), delta, pool_size);
    // Bounding phase
    auto start = steady_clock::now();
    pulse_algorithm.bound();
    auto end = steady_clock::now();
    int duration_bound = duration_cast<milliseconds>(end - start).count();

    // Solve the pricing problem
    auto start_pricing = steady_clock::now();
    int error = pulse_algorithm.solve(vehicle.cost, dual_solution.betas[vehicle.id]);

    if (error != 0) {
        cout << "Error in solving the pulse algorithm" << endl;
        return {};
    }

    // Transform the partial pathes from the solution pool into Route objects
    std::vector<Route> new_routes;
        
    for (const auto& [rc, path] : pulse_algorithm.get_solution_pool()) {
        new_routes.push_back(convert_sequence_to_route(rc, path.sequence, instance, vehicle));
    }
    auto end_pricing = steady_clock::now();
    int duration_pricing = duration_cast<milliseconds>(end_pricing - start_pricing).count();

    cout << "V" << vehicle.id << " : Bound in " << duration_bound << " ms, pricing in " << duration_pricing << " ms" << endl;

    return new_routes;
}


std::vector<Route> solve_pricing_problem_pulse_grouped(
    const Instance &instance, 
    const std::vector<int> & vehicle_indexes,
    const DualSolution &dual_solution,
    int delta,
    int pool_size
) {
    using std::vector, std::set, std::map;
    using namespace std::chrono;

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
    int n_interventions_v = all_interventions_v.size();

    // Next step : create the pricing problem
    unique_ptr<Problem> pricing_problem = create_pricing_instance(instance, virtual_vehicle);
    // Update the pricing problem with the dual values
    update_pricing_instance(pricing_problem, dual_solution, instance, virtual_vehicle);

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
        // Get the available interventions
        vector<int> available_interventions(n_interventions_v, 0);
        for (int i : vehicle.interventions) {
            available_interventions[virtual_vehicle.reverse_interventions[i]] = 1;
        }
        // Solve the pricing problem
        int error = pulse_algorithm.solve(vehicle.cost, dual_solution.betas[v], available_interventions);
        if (error != 0) {
            cout << "Error in solving the pulse algorithm for vehicle " << v << endl;
            continue;
        }
        // Transform the partial pathes from the solution pool into Route objects
        for (const auto& [rc, path] : pulse_algorithm.get_solution_pool()) {
            Route route = convert_sequence_to_route(rc, path.sequence, instance, virtual_vehicle);
            // Update the route with the real vehicle id & add the cost of the vehicle
            route.vehicle_id = v;
            route.total_cost += vehicle.cost;
            new_routes.push_back(route);
        }
    }
    auto end_pricing = steady_clock::now();
    int duration_pricing = duration_cast<milliseconds>(end_pricing - start_pricing).count();

    cout << "Group [";
    for (int v : vehicle_indexes) {
        cout << "v" << v << ", ";
    }
    cout << "] : Bound in " << duration_bound << " ms, pricing in " << duration_pricing << " ms" << endl;

    return new_routes;
}


std::vector<Route> solve_pricing_problem_pulse_parallel(
    const Instance &instance, 
    const Vehicle &vehicle,
    const DualSolution &dual_solution,
    int delta,
    int pool_size,
    const std::set<std::tuple<int, int, int>> &forbidden_edges,
    const std::set<std::tuple<int, int, int>> &required_edges
    ) {
    using namespace std::chrono;
    // Create the pricing problem
    unique_ptr<Problem> pricing_problem = create_pricing_instance(instance, vehicle, true, forbidden_edges, required_edges);
    // Update the pricing problem with the dual values
    update_pricing_instance(pricing_problem, dual_solution, instance, vehicle);
    // Create the pulse algorithm
    PulseAlgorithm pulse_algorithm = PulseAlgorithm(pricing_problem.get(), delta, pool_size);
    // Bounding phase
    auto start = steady_clock::now();
    pulse_algorithm.bound_parallel();
    auto end = steady_clock::now();
    int duration_bound = duration_cast<milliseconds>(end - start).count();

    // Solve the pricing problem
    auto start_pricing = steady_clock::now();
    int error = pulse_algorithm.solve_parallel(vehicle.cost, dual_solution.betas[vehicle.id]);

    if (error != 0) {
        cout << "Error in solving the pulse algorithm" << endl;
        return {};
    }

    // Transform the partial pathes from the solution pool into Route objects
    std::vector<Route> new_routes;
        
    for (const auto& [rc, path] : pulse_algorithm.get_solution_pool()) {
        new_routes.push_back(convert_sequence_to_route(rc, path.sequence, instance, vehicle));
    }
    auto end_pricing = steady_clock::now();
    int duration_pricing = duration_cast<milliseconds>(end_pricing - start_pricing).count();

    cout << "V" << vehicle.id << " : Bound in " << duration_bound << " ms, pricing in " << duration_pricing << " ms" << endl;

    return new_routes;
}

