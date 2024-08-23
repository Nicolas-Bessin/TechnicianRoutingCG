#include "route.h"
#include "instance.h"

// Optimize a route for travel distance
Route optimize_route(const Route& route, const Instance& instance);


// Optimizes the routes in a given integer solution and returns the new integer solution
// Also updates the routes vector with the optimized routes
IntegerSolution optimize_routes(const IntegerSolution& integer_solution, std::vector<Route>& routes, const Instance& instance);