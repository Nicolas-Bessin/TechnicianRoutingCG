#ifndef PROBLEM_H
#define PROBLEM_H

#include "resources/defaultcost.h"
#include "resources/resource.h"
#include "resources/capacity.h"
#include "resources/time.h"
#include "resources/node_limit.h"
#include "resources/time_windows.h"
#include "resource_data.h"
#include "utils/param.h"
#include "utils/data_collector.h"
#include "graph.h"
#include "bound_data.h"
#include "utils/bitset.h"

class Problem {

public:

    /** Problem Management **/
    //Constructors and destructors
    Problem();
    // Custom constructor
    Problem(std::string name, int n_nodes, int origin, int destination, int n_res, bool cycles, bool complete, bool directed);
    ~Problem();

    //Problem setup
    void initProblem();

    //Name
    void setName(std::string name) {this->name = name;}
    std::string getName() {return name;}

    //Status
    int getStatus() {return problem_status;}
    void setStatus(int status) {problem_status = status;}
    void printStatus();

    //Number of nodes
    int getNumNodes() const {return n_nodes;}

    //Origin and Destination
    void setOrigin(int origin){ this->origin = origin;}
    int getOrigin() const {return origin;} 
    void setDestination(int destination){ this->destination = destination;}
    int getDestination() const {return destination;}

    //Cycles
    bool isGraphCyclic(){return cycles;}
    void setGraphCycles(bool cycles)  {this->cycles = cycles;}

    //Memory saving information
    bool isDataCompressed(){return compress_data;}

    /** Graph management**/
    //Custom method : Add an arc to the underlying graph
    void setNetworkArc(int i, int j);
    //Get distance (coordinate based)
    int getCoordDistance(int i, int j) {return network.getCoordDistance(i, j);}

    //Neighbors
    bool areNeighbors(int i, int j, bool direction) {return network.areNeighbors(i, j, direction);}
    std::vector<int> & getNeighbors(int node, bool direction) {return network.getNeighbors(node, direction);}

    //Active nodes
    bool isActiveNode(int i) {return network.isActiveNode(i);}
    int countActiveNodes() {return network.countActiveNodes();}

    void activateNode(int i) {network.activateNode(i);}

    void pruneActiveNode(int i) {network.pruneActiveNode(i);}
    void pruneUnreachableNodes(Bitset & reachable) {network.pruneUnreachableNodes(reachable);}

    void resetActiveNodes() {network.resetActiveNodes();}

    /** Objective and Resource management **/
    //Objective initialization
    void initObjective(Resource<double>* objective = nullptr);

    // Custom : objective setter
    void setObjective(Resource<double>* objective) {this->objective = objective;}

    //Resource management
    int addResource(int type);
    void createResources(std::vector<int> & resources_type);
    void setRes(Resource<int>* res){resources.push_back(res); n_res++;}
    void setRes(int position, Resource<int>* res){resources[position] = res;}
    void setResources(std::vector<Resource<int>*> resources){this->resources = resources; n_res = resources.size();}

    //Insert sets of consumptions f
    void setArcConsumption(int i, int j,  std::vector<int> consumption);
    void setNodeConsumption(int id, std::vector<int> consumption);

    //Resource/Objective data queries
    int getNumRes(){return resources.size();}
    Resource<double>* getObj() {return objective;}
    Resource<int>* getRes(int position){return resources[position];}
    std::vector<Resource<int>*> & getResources(){return resources;}

    /** Completion Labels management  **/
    //Completion Labels data queries
    void initBoundLabels(){bound_labels = new BoundLabels(n_res, n_nodes);}
    BoundLabels* getBoundLabels(){return bound_labels;}
    void resetBoundLabels(){delete bound_labels; initBoundLabels();}

    //Scaling
    void scaleObjective(float scaling);
    void scaleResource(int id, float scaling);
    void scaleAllData(float scaling);

    /** Read from instance file **/
    //Read standard input problem and print problem data
    virtual void readProblem(std::string file_name);
    void readSparseProblem(std::string file_name);
    void getTokens(std::string & line, std::vector<std::string> & tokens);
    bool readNextLine(std::ifstream & f, std::string & line, std::vector<std::string> & tokens, std::string & key);

    /** Output management **/
    void printProblem();

    /** Data collection **/
    void initDataCollection();
    void collectData();

protected:

    std::string name;
    int problem_status;

    //Graph information
    Graph network;
    bool directed;
    bool complete;
    bool cycles;
    bool compress_data;

    //Node information
    int n_nodes;
    int origin, destination;

    //Arc information Data
    int n_arcs;

    //Objective and Resources
    int n_res;
    Resource<double>* objective;
    std::vector<Resource<int>*> resources;

    //Completion Labels
    BoundLabels* bound_labels;

    //Data collection
    DataCollector collector;

};

#endif
