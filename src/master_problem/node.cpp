#include "node.h"

#include <vector>


BPNode RootNode(const std::vector<Route>& initial_routes){
    BPNode root;
    root.depth = 0;
    root.upper_bound = 0;
    for (int i = 0; i < initial_routes.size(); i++){
        root.active_routes.insert(i);
    }
    return root;
}