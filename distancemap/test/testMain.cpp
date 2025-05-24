#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <cmath>
#include <algorithm>
#include <sstream> // For string conversion in assertions

#include "GridToGraph.hpp"
#include "GridTypes.hpp"

// --- Minimal Assertion Framework ---
int tests_passed = 0;
int tests_failed = 0;
const int INF_TEST = std::numeric_limits<int>::max();

// Helper to convert values to string for assertion messages
template <typename T>
std::string to_str(const T& value) {
    std::ostringstream oss;
    oss << value;
    return oss.str();
}
// Specialization for INF_TEST to print "INF"
template <>
std::string to_str<int>(const int& value) {
    if (value == INF_TEST) {
        return "INF";
    }
    return std::to_string(value);
}


void ASSERT_TRUE(bool condition, const std::string& message) {
    if (condition) {
        tests_passed++;
    } else {
        tests_failed++;
        std::cerr << "[FAIL] Test: " << message << std::endl;
    }
}

template <typename T>
void ASSERT_EQ(const T& actual, const T& expected, const std::string& message) {
    if (actual == expected) {
        tests_passed++;
    } else {
        tests_failed++;
        std::cerr << "[FAIL] Test: " << message << "\n"
                  << "       Expected: " << to_str(expected) << "\n"
                  << "       Actual:   " << to_str(actual) << std::endl;
    }
}

template <typename T>
void ASSERT_MATRIX_EQ(const std::vector<std::vector<T>>& actual, 
                      const std::vector<std::vector<T>>& expected, 
                      const std::string& message) {
    bool overall_equal = true;
    if (actual.size() != expected.size()) {
        overall_equal = false;
    } else {
        for (size_t i = 0; i < actual.size(); ++i) {
            if (expected.empty() && !actual[i].empty()) { // Expected row is empty, actual is not
                 overall_equal = false;
                 break;
            }
            if (!expected.empty() && (actual[i].size() != expected[i].size())) {
                overall_equal = false;
                break;
            }
             if (expected.empty() && actual[i].empty()){ // both empty rows, continue
                continue;
            }
            for (size_t j = 0; j < actual[i].size(); ++j) {
                if (actual[i][j] != expected[i][j]) {
                    overall_equal = false;
                    break;
                }
            }
            if (!overall_equal) break;
        }
    }

    if (overall_equal) {
        tests_passed++;
    } else {
        tests_failed++;
        std::cerr << "[FAIL] Test: " << message << "\n"
                  << "       Expected and Actual matrices differ." << std::endl;
        // Simple print for small matrices
        auto print_matrix = [&](const std::vector<std::vector<T>>& matrix, const std::string& name) {
            size_t rows = matrix.size();
            size_t cols = rows > 0 ? matrix[0].size() : 0;
            if (rows < 5 && (rows == 0 || cols < 5)) { // Check if matrix is small
                 std::cerr << name << " Matrix (" << rows << "x" << cols << "):\n";
                for (const auto& row : matrix) {
                    for (const auto& val : row) { std::cerr << to_str(val) << "\t"; }
                    std::cerr << "\n";
                }
            }
        };
        print_matrix(actual, "Actual");
        print_matrix(expected, "Expected");
    }
}

// --- Test Function Shells ---
void testComputeAllPathDists_tests(); 
void testCreateAbstractNodes_tests(); 

// --- Main Function ---
int main(int argc, char** argv) {
    std::cout << "Starting unit tests..." << std::endl;

    testComputeAllPathDists_tests();
    testCreateAbstractNodes_tests();

    std::cout << "\n--- Test Summary ---" << std::endl;
    std::cout << "Tests Run:    " << tests_passed + tests_failed << std::endl;
    std::cout << "Tests Passed: " << tests_passed << std::endl;
    std::cout << "Tests Failed: " << tests_failed << std::endl;

    if (tests_failed > 0) {
        std::cerr << "\nSOME TESTS FAILED!" << std::endl;
        return 1; 
    }
    
    std::cout << "\nAll tests passed." << std::endl;
    return 0; 
}

// --- Test Function Implementations (Shells for now, will be filled) ---
GridType::Edge create_edge_test_helper(int from, int to, int weight, bool toDeadEnd = false) {
    GridType::Edge edge = {from, to, toDeadEnd};
    if (weight > 0) {
       edge.path.resize(weight); 
    }
    return edge;
}

void testComputeAllPathDists_tests() {
    std::cout << "\n--- Testing GridToGraph::computeAllPathDists ---" << std::endl;
    // Scenario 1: Simple Connected Graph
    {
        std::vector<GridType::Edge> edges = {
            create_edge_test_helper(0, 1, 10),
            create_edge_test_helper(0, 2, 20),
            create_edge_test_helper(1, 2, 5) 
        };
        int numNodes = 3;
        std::vector<std::vector<int>> expected_dists = {
            {0, 10, 15},
            {10, 0, 5},
            {15, 5, 0}
        };
        std::vector<std::vector<int>> actual_dists = GridToGraph::computeAllPathDists(edges, numNodes);
        ASSERT_MATRIX_EQ(actual_dists, expected_dists, "S1: Simple Connected Graph");
    }
     // Scenario 2: Disconnected Components
    {
        std::vector<GridType::Edge> edges = {
            create_edge_test_helper(0, 1, 1), 
            create_edge_test_helper(2, 3, 2)  
        };
        int numNodes = 4;
        std::vector<std::vector<int>> expected_dists = {
            {0, 1, INF_TEST, INF_TEST},
            {1, 0, INF_TEST, INF_TEST},
            {INF_TEST, INF_TEST, 0, 2},
            {INF_TEST, INF_TEST, 2, 0}
        };
        std::vector<std::vector<int>> actual_dists = GridToGraph::computeAllPathDists(edges, numNodes);
        ASSERT_MATRIX_EQ(actual_dists, expected_dists, "S2: Disconnected Components");
    }
    // Scenario 3: Single Node
    {
        std::vector<GridType::Edge> edges = {};
        int numNodes = 1;
        std::vector<std::vector<int>> expected_dists = {{0}};
        std::vector<std::vector<int>> actual_dists = GridToGraph::computeAllPathDists(edges, numNodes);
        ASSERT_MATRIX_EQ(actual_dists, expected_dists, "S3: Single Node");
    }
    // Scenario 4: Multiple Nodes, No Edges
    {
        std::vector<GridType::Edge> edges = {};
        int numNodes = 3;
        std::vector<std::vector<int>> expected_dists = {
            {0, INF_TEST, INF_TEST},
            {INF_TEST, 0, INF_TEST},
            {INF_TEST, INF_TEST, 0}
        };
        std::vector<std::vector<int>> actual_dists = GridToGraph::computeAllPathDists(edges, numNodes);
        ASSERT_MATRIX_EQ(actual_dists, expected_dists, "S4: Multiple Nodes, No Edges");
    }
    // Scenario 5: Empty Graph (Zero Nodes)
    {
        std::vector<GridType::Edge> edges = {};
        int numNodes = 0;
        std::vector<std::vector<int>> expected_dists = {}; 
        std::vector<std::vector<int>> actual_dists = GridToGraph::computeAllPathDists(edges, numNodes);
        ASSERT_MATRIX_EQ(actual_dists, expected_dists, "S5: Empty Graph (0 Nodes)");
    }
    // Scenario 6: More complex graph with a longer path being shorter
    {
        std::vector<GridType::Edge> edges = {
            create_edge_test_helper(0, 1, 1),
            create_edge_test_helper(1, 2, 1),
            create_edge_test_helper(2, 3, 1), 
            create_edge_test_helper(0, 3, 10) 
        };
        int numNodes = 4;
         std::vector<std::vector<int>> expected_dists = {
            {0, 1, 2, 3},
            {1, 0, 1, 2},
            {2, 1, 0, 1},
            {3, 2, 1, 0}
        };
        std::vector<std::vector<int>> actual_dists = GridToGraph::computeAllPathDists(edges, numNodes);
        ASSERT_MATRIX_EQ(actual_dists, expected_dists, "S6: Longer path is shorter");
    }
}

void testCreateAbstractNodes_tests() {
    std::cout << "\n--- Testing GridToGraph::createAbstractNodes ---" << std::endl;
    using namespace GridType;

    auto check_abstract_node_contents = [](const AbstractNode& an, const std::vector<int>& expected_base_nodes_param, const std::string& msg_prefix) {
        std::vector<int> expected_base_nodes_sorted = expected_base_nodes_param;
        std::sort(expected_base_nodes_sorted.begin(), expected_base_nodes_sorted.end());

        ASSERT_EQ(an.baseNodes.size(), expected_base_nodes_sorted.size(), msg_prefix + " baseNodes count");
        
        std::vector<int> sorted_actual_bn = an.baseNodes;
        std::sort(sorted_actual_bn.begin(), sorted_actual_bn.end());
        
        bool match = sorted_actual_bn == expected_base_nodes_sorted;
        ASSERT_TRUE(match, msg_prefix + " baseNodes content match");
        if (!match) {
            std::cerr << "      Actual sorted: "; for(int id : sorted_actual_bn) std::cerr << id << " "; std::cerr << std::endl;
            std::cerr << "      Expected sorted: "; for(int id : expected_base_nodes_sorted) std::cerr << id << " "; std::cerr << std::endl;
        }

        if (!an.baseNodes.empty()) {
            bool found_center_in_list = (std::find(an.baseNodes.begin(), an.baseNodes.end(), an.baseCenterNode) != an.baseNodes.end());
            ASSERT_TRUE(found_center_in_list, msg_prefix + " baseCenterNode (" + std::to_string(an.baseCenterNode) + ") is in its baseNodes list");
        } else {
            ASSERT_EQ(an.baseCenterNode, -1, msg_prefix + " baseCenterNode for empty baseNodes is -1");
        }
    };

    // Scenario 1: Distinct Clusters
    {
        std::vector<Point> nodes = {{0,0}, {1,0}, {0,1},      // Clust A (idx 0,1,2)
                                    {10,10}, {11,10}, {10,11}}; // Clust B (idx 3,4,5)
        double eps = 2.0;
        int minClusterSize = 3;
        std::vector<AbstractNode> actual_abstract_nodes = GridToGraph::createAbstractNodes(nodes, eps, minClusterSize);
        ASSERT_EQ(actual_abstract_nodes.size(), (size_t)2, "S1: Distinct Clusters - Count");

        if (actual_abstract_nodes.size() == 2) {
            std::vector<int> expected_c1_nodes = {0,1,2}; 
            std::vector<int> expected_c2_nodes = {3,4,5}; 
            
            std::vector<int> actual_an0_nodes_sorted = actual_abstract_nodes[0].baseNodes;
            std::sort(actual_an0_nodes_sorted.begin(), actual_an0_nodes_sorted.end());
            std::vector<int> actual_an1_nodes_sorted = actual_abstract_nodes[1].baseNodes;
            std::sort(actual_an1_nodes_sorted.begin(), actual_an1_nodes_sorted.end());

            bool test_passed_s1 = false;
            if (actual_an0_nodes_sorted == expected_c1_nodes && actual_an1_nodes_sorted == expected_c2_nodes) {
                 check_abstract_node_contents(actual_abstract_nodes[0], expected_c1_nodes, "S1: Cluster A (match order 1)");
                 check_abstract_node_contents(actual_abstract_nodes[1], expected_c2_nodes, "S1: Cluster B (match order 1)");
                 test_passed_s1 = true;
            } else if (actual_an0_nodes_sorted == expected_c2_nodes && actual_an1_nodes_sorted == expected_c1_nodes) {
                 check_abstract_node_contents(actual_abstract_nodes[0], expected_c2_nodes, "S1: Cluster B (match order 2)");
                 check_abstract_node_contents(actual_abstract_nodes[1], expected_c1_nodes, "S1: Cluster A (match order 2)");
                 test_passed_s1 = true;
            }
            ASSERT_TRUE(test_passed_s1, "S1: Both distinct clusters found and validated");
        }
    }

    // Scenario 2: All Noise
    {
        std::vector<Point> nodes = {{0,0}, {100,100}, {0,200}};
        double eps = 5.0;
        int minClusterSize = 2; 
        std::vector<AbstractNode> abstract_nodes = GridToGraph::createAbstractNodes(nodes, eps, minClusterSize);
        ASSERT_EQ(abstract_nodes.size(), (size_t)0, "S2: All Noise - Count");
    }

    // Scenario 3: Single Cluster
    {
        std::vector<Point> nodes = {{0,0}, {1,0}, {0,1}, {1,1}, {2,1}};
        double eps = 2.0; 
        int minClusterSize = 3;
        std::vector<AbstractNode> abstract_nodes = GridToGraph::createAbstractNodes(nodes, eps, minClusterSize);
        ASSERT_EQ(abstract_nodes.size(), (size_t)1, "S3: Single Cluster - Count");
        if (abstract_nodes.size() == 1) {
            check_abstract_node_contents(abstract_nodes[0], {0,1,2,3,4}, "S3: Single Cluster - Content");
        }
    }
    
    // Scenario 4: Empty Input
    {
        std::vector<Point> nodes = {};
        double eps = 2.0;
        int minClusterSize = 2;
        std::vector<AbstractNode> abstract_nodes = GridToGraph::createAbstractNodes(nodes, eps, minClusterSize);
        ASSERT_EQ(abstract_nodes.size(), (size_t)0, "S4: Empty Input - Count");
    }

    // Scenario 5: One Point
    {
        std::vector<Point> nodes = {{10,20}};
        double eps = 2.0;
        int minClusterSize = 1; 
        std::vector<AbstractNode> abstract_nodes = GridToGraph::createAbstractNodes(nodes, eps, minClusterSize);
        ASSERT_EQ(abstract_nodes.size(), (size_t)1, "S5a: One Point, minPts=1 - Count");
        if(abstract_nodes.size()==1) check_abstract_node_contents(abstract_nodes[0], {0}, "S5a: One Point, minPts=1 - Content");

        minClusterSize = 2; 
        abstract_nodes = GridToGraph::createAbstractNodes(nodes, eps, minClusterSize);
        ASSERT_EQ(abstract_nodes.size(), (size_t)0, "S5b: One Point, minPts=2 (noise) - Count");
    }
    
    // Scenario 6: Two Points
    {
        std::vector<Point> nodes_close = {{0,0}, {1,0}}; 
        double eps = 1.5;
        int minClusterSize = 2;
        std::vector<AbstractNode> abstract_nodes_close = GridToGraph::createAbstractNodes(nodes_close, eps, minClusterSize);
        ASSERT_EQ(abstract_nodes_close.size(), (size_t)1, "S6a: Two Close Points, minPts=2 - Count");
        if(abstract_nodes_close.size()==1) check_abstract_node_contents(abstract_nodes_close[0], {0,1}, "S6a: Two Close Points - Content");

        std::vector<Point> nodes_far = {{0,0}, {10,0}}; 
        std::vector<AbstractNode> abstract_nodes_far = GridToGraph::createAbstractNodes(nodes_far, eps, minClusterSize);
        ASSERT_EQ(abstract_nodes_far.size(), (size_t)0, "S6b: Two Far Points, minPts=2 (noise) - Count");
    }
}