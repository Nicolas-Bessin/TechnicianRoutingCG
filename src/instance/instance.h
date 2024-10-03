// Definition of the various classes used to represent an instance of the problem
#pragma once
#include <map>
#include <vector>
#include <string>
#include <set>

#include "instance/constants.h"

#define KEEP_NON_COVERED 0
#define KEEP_COVERED 1

/*
    @struct Node
    @brief Represents either an intervention or a warehouse in the problem
*/
struct Node {
    // id is a unique string identifier for the node
    std::string id;
    // node_id is the index of the node in the time and distance matrice
    int node_id;
    // Type of the node
    bool is_intervention;
    // Duration of the intervention
    int duration;
    // Time window for the intervention
    int start_window;
    int end_window;
    // Can the intervention be done both in the morning or in the afternoon?
    bool is_ambiguous;
    // Quantities of each ressource used by the intervention
    // The ressources help prevent using up all the time doing interventions (to leave flexibility in the planning)
    std::map<std::string, int> quantities;
    // Skills required to perform the intervention (left empty for warehouses) (counting the number of technicians needed with each skill)
    std::map<std::string, int> required_skills;
    // Position information
    std::pair<double, double> position;

    // Empty constructor
    Node(){}

    // Constructor for a warehouse
    Node(std::string id, int node_id, std::pair<double, double> pos) : 
        id(id), node_id(node_id), is_intervention(false), duration(0), start_window(0), end_window(END_DAY), position(pos) {}

    // Constructor for an intervention
    Node(
        std::string id,
        int node_id,
        int duration,
        int start_window,
        int end_window,
        bool is_ambiguous,
        std::map<std::string, int> quantities,
        std::map<std::string, int> required_skills,
        std::pair<double, double> pos
    ) : 
        id(id), 
        node_id(node_id), 
        is_intervention(true), 
        duration(duration), 
        start_window(start_window), 
        end_window(end_window), 
        is_ambiguous(is_ambiguous), 
        quantities(quantities), 
        required_skills(required_skills),
        position(pos)
        {}
};

/*
    @struct Technician
    @brief Represents a technician in the problem
*/
struct Technician {
    std::string id;
    std::string operationnal_base;
    std::set<std::string> skills;
    std::map<std::string, int> capacities;

    // Empty constructor
    Technician(){}

    // Constructor for a technician
    Technician(std::string id, std::string operationnal_base, std::set<std::string> skills, std::map<std::string, int> capacities) :
        id(id), operationnal_base(operationnal_base), skills(skills), capacities(capacities) {}
};

/*
    @struct Vehicle
    @brief Represents a vehicle in the problem
*/
struct Vehicle {
    int id;
    // Vector containing the ids of the technicians in the vehicle
    std::vector<std::string> technicians;
    // Counts the number of technicians that have each skill in the vehicle
    std::map<std::string, int> skills; 
    // List of indexes of interventions that the vehicle can perform
    // (indexes with respect to the nodes std::vector in the instance)
    std::vector<int> interventions;
    // Reverse index of the interventions (from the nodes std::vector to the interventions vector)
    std::map<int, int> reverse_interventions;
    // Index of the depot in the nodes std::vector
    int depot;
    std::map<std::string, int> capacities;
    double cost;

    // Empty constructor
    Vehicle(){}

    // Constructor for a vehicle
    Vehicle(
        int id,
        std::vector<std::string> technicians,
        std::map<std::string, int> skills,
        std::vector<int> interventions,
        std::map<int, int> reverse_interventions,
        int depot,
        std::map<std::string, int> capacities,
        double cost
    ) :
        id(id),
        technicians(technicians),
        skills(skills),
        interventions(interventions),
        reverse_interventions(reverse_interventions),
        depot(depot),
        capacities(capacities),
        cost(cost)
        {}
};

// Generate a new vehicle from an existing one
// @param mode : KEEP_NON_COVERED or KEEP_COVERED
//
// KEEP_NON_COVERED : the new vehicle will only have the interventions set to 0 in the mask 
//
// KEEP_COVERED : the new vehicle will only have the interventions set to 1 in the mask
Vehicle vehicle_mask(const Vehicle& vehicle, const std::vector<int>& mask, bool mode = KEEP_NON_COVERED);


/*
    @struct Instance
    @brief Represents an instance of the problem
*/
struct Instance {
    std::string name;
    int number_interventions;
    int number_warehouses;
    int number_vehicles;
    double cost_per_km;
    double technician_cost;
    // Big M used in the objective function
    double M;
    // std::vector of interventions and warehouses
    std::vector<Node> nodes;
    // std::vector of vehicles
    std::vector<Vehicle> vehicles;
    // Different capacities considered
    std::vector<std::string> capacities_labels;
    // time between the different nodes
    std::vector<std::vector<int>> time_matrix;
    // distance between the different nodes
    std::vector<std::vector<int>> distance_matrix;
    // Hamming similarity between vehicles (the lower the more similar)
    std::vector<std::vector<int>> similarity_matrix;
    // Instance constructor
    Instance(
        std::string name,
        int number_interventions,
        int number_warehouses,
        int number_vehicles,
        double cost_per_km,
        double technician_cost,
        double M,
        std::vector<Node> nodes,
        std::vector<Vehicle> vehicles,
        std::vector<std::string> capacities_labels,
        std::vector<std::vector<int>> time_matrix,
        std::vector<std::vector<int>> distance_matrix,
        std::vector<std::vector<int>> similarity_matrix
    ) :
        name(name),
        number_interventions(number_interventions),
        number_warehouses(number_warehouses),
        number_vehicles(number_vehicles),
        cost_per_km(cost_per_km),
        technician_cost(technician_cost),
        M(M),
        nodes(nodes),
        vehicles(vehicles),
        capacities_labels(capacities_labels),
        time_matrix(time_matrix),
        distance_matrix(distance_matrix),
        similarity_matrix(similarity_matrix)
        {}
};


bool is_symmetric(const std::vector<std::vector<int>>& matrix);

int symmetry_gap(const std::vector<std::vector<int>>& matrix);

bool can_do_intervention(const Node& intervention, const Vehicle& vehicle);


// Checks wether intervention j can follow intervention i (is the edge i->j feasible?)
// We check this by looking at the time window of the interventions
bool is_edge_feasible(int i, int j, const Instance& instance);


// Check the triangular inequality for the distance and time matrices
void check_triangular_inequality(const Instance& instance);


// Compute the naive oursourcing cost M for a given instance
double compute_M_naive(const Instance& instance);


// Compute the oursourcing cost M for a given instance
// Use the "per vehicle" formulation
double compute_M_perV(const Instance& instance);
