#include "parameters.h"


ColumnGenerationParameters::ColumnGenerationParameters(std::map<std::string, std::any> args) {

    // General parameters
    if (args.contains("time_limit"))
        time_limit = std::any_cast<int>(args["time_limit"]);

    if (args.contains("reduced_cost_threshold"))
        reduced_cost_threshold = std::any_cast<double>(args["reduced_cost_threshold"]);

    if (args.contains("verbose"))
        verbose = std::any_cast<bool>(args["verbose"]);

    if (args.contains("max_iterations"))
        max_iterations = std::any_cast<int>(args["max_iterations"]);

    if (args.contains("max_consecutive_non_improvement"))
        max_consecutive_non_improvement = std::any_cast<int>(args["max_consecutive_non_improvement"]);

    if (args.contains("compute_integer_solution"))
        compute_integer_solution = std::any_cast<bool>(args["compute_integer_solution"]);

    if (args.contains("use_maximisation_formulation"))
        use_maximisation_formulation = std::any_cast<bool>(args["use_maximisation_formulation"]);

    if (args.contains("compute_intermediate_integer_solutions"))
        compute_intermediate_integer_solutions = std::any_cast<bool>(args["compute_intermediate_integer_solutions"]);

    // Pathwyse related parameters
    if (args.contains("max_resources_dominance")) {
        max_resources_dominance = std::any_cast<int>(args["max_resources_dominance"]);
    }
    if (args.contains("switch_to_cyclic_pricing")) {
        switch_to_cyclic_pricing = std::any_cast<bool>(args["switch_to_cyclic_pricing"]);
    }
    if (args.contains("use_visited"))
        use_visited = std::any_cast<bool>(args["use_visited"]);
    if (args.contains("ng"))
        ng = std::any_cast<int>(args["ng"]);
    if (args.contains("dssr"))
        dssr = std::any_cast<int>(args["dssr"]);
    if (args.contains("pathwyse_time_limit"))
        pathwyse_TL = std::any_cast<double>(args["pathwyse_time_limit"]);
    if (args.contains("bidirectional_DP"))
        bidirectional_DP = std::any_cast<bool>(args["bidirectional_DP"]);

    // Pulse related parameters
    if (args.contains("delta")) {
        delta = std::any_cast<int>(args["delta"]);
    }
    if (args.contains("solution_pool_size")) {
        solution_pool_size = std::any_cast<int>(args["solution_pool_size"]);
    }

    // Stabilisation parameters
    if (args.contains("alpha")) {
        alpha = std::any_cast<double>(args["alpha"]);
    }
    if (args.contains("use_stabilisation")) {
        use_stabilisation = std::any_cast<bool>(args["use_stabilisation"]);
    }

    // Pricing function
    if (args.contains("pricing_function")) {
        pricing_function = std::any_cast<std::string>(args["pricing_function"]);
    }

    if (args.contains("pricing_verbose")) {
        pricing_verbose = std::any_cast<bool>(args["pricing_verbose"]);
    }
}


BranchAndPriceParameters::BranchAndPriceParameters(std::map<std::string, std::any> args) : ColumnGenerationParameters(args) {
    // Added parameters for the branch and price algorithm
    if (args.contains("max_depth")) {
        max_depth = std::any_cast<int>(args["max_depth"]);
    }
    if (args.contains("time_limit_per_node")) {
        time_limit_per_node = std::any_cast<int>(args["time_limit_per_node"]);
    }
}