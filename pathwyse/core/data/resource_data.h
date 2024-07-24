#ifndef RESOURCE_DATA_H
#define RESOURCE_DATA_H

#include <vector>
#include <map>

template <typename T>
struct ResourceData {

    /** Resource data management **/
    //Constructors and Destructors
    explicit ResourceData(int n_nodes) {this->n_nodes = n_nodes;}
    virtual ~ResourceData() = default;

    virtual void reset() {node_costs.clear();};

    /** Node Data **/
    void setNodeCosts(std::vector<T> node_costs){this->node_costs = node_costs;}
    void setNodeCost(int id, T cost){
        if(node_costs.empty())
            node_costs.resize(n_nodes, 0);
        node_costs[id] = cost;
    }

    void increaseNodeCost(int id, T delta) {
        if(not node_costs.empty())
            node_costs[id] += delta;
    }

    void multiplyNodeCost(int id, float factor) {
        if(not node_costs.empty())
            node_costs[id] *= factor;
    }

    T getNodeCost(int id){return node_costs.empty() ? 0: node_costs[id];}
    std::vector<T> & getNodeCost() {return node_costs;}

    /** Arc Data **/
    virtual void setArcCost(int i, int j, T cost) = 0;
    virtual void increaseArcCost(int i, int j, T delta) = 0;
    virtual void multiplyArcCost(int i, int j, float factor) = 0;
    virtual T getArcCost(int i, int j) = 0;

    /** Scaling **/
    virtual void scaleData(float scaling) = 0;

protected:
    int n_nodes;
    std::vector<T> node_costs;
};

template <typename T>
struct ResourceDataMap: ResourceData<T> {

    /** Resource data management **/
    //Constructors and Destructors
    explicit ResourceDataMap(int n_nodes): ResourceData<T>(n_nodes) {
        arc_costs.resize(n_nodes, std::map<int, T>());
    }
    ~ResourceDataMap() = default;

    void reset() override {ResourceData<T>::reset(); arc_costs.clear();}

    /** Arc Data **/
    void setArcCost(int i, int j, T cost) override {
        auto position = arc_costs[i].find(j);
        if(position == arc_costs[i].end())
            arc_costs[i].insert(std::make_pair(j, cost));
        else
            position->second = cost;
    }
    void increaseArcCost(int i, int j, T delta) override {
        auto position = arc_costs[i].find(j);
        if(position != arc_costs[i].end())
            position->second += delta;
    }

    void multiplyArcCost(int i, int j, float factor) override {
        auto position = arc_costs[i].find(j);
        if(position != arc_costs[i].end())
            position->second *= factor;
    }

    T getArcCost(int i, int j) override {
        auto position = arc_costs[i].find(j);
        return position != arc_costs[i].end()? position->second : 0;
    }

    /** Scaling **/
    void scaleData(float scaling) override{
        int n_nodes = ResourceData<T>::n_nodes;
        for(int i = 0; i < n_nodes; i++) {
            ResourceData<T>::multiplyNodeCost(i, scaling);
            for(auto & it : arc_costs[i])
                it.second *= scaling;
        }
    }

private:
    std::vector<std::map<int, T>> arc_costs;

};

template <typename T>
struct ResourceDataMatrix: ResourceData<T> {

    /** Resource data management **/
    //Constructors and Destructors
    explicit ResourceDataMatrix(int n_nodes): ResourceData<T>(n_nodes) {
        arc_costs.resize(n_nodes, std::vector<T>(n_nodes, 0));
    }
    ~ResourceDataMatrix() = default;

    void reset() override {ResourceData<T>::reset(); arc_costs.clear();}

    /** Arc Data **/
    void setArcCost(int i, int j, T cost) override {arc_costs[i][j] = cost;}
    T getArcCost(int i, int j) override {return arc_costs[i][j];}
    void increaseArcCost(int i, int j, T delta) override {arc_costs[i][j] += delta;}
    void multiplyArcCost(int i, int j, float factor) override {arc_costs[i][j] *= factor;}

    /** Scaling **/
    void scaleData(float scaling) override{
        int n_nodes = ResourceData<T>::n_nodes;
        for(int i = 0; i < n_nodes; i++){
            ResourceData<T>::multiplyNodeCost(i, scaling);
            for(int j = 0; j < n_nodes; j++)
                multiplyArcCost(i, j, scaling);
        }
    }

private:
    std::vector<std::vector<T>> arc_costs;
};

#endif
