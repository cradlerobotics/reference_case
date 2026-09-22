#include <global_planner/wavefront.h>
#include <algorithm>
#include <queue>
#include <vector>
#include <cmath>

namespace global_planner {

// Helper struct for Priority Queue
struct Node {
    int index;
    float cost;
    
    // Operator to make priority_queue act as a Min-Heap (lowest cost first)
    bool operator>(const Node& other) const {
        return cost > other.cost;
    }
};

WavefrontExpansion::WavefrontExpansion(PotentialCalculator* p_calc, int nx, int ny) :
        Expander(p_calc, nx, ny) {
}

bool WavefrontExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                           int cycles, float* potential) {
    
    int size = nx_ * ny_;
    // 1. Initialize potentials to Infinity
    std::fill(potential, potential + size, 1e10);

    // 2. Setup Priority Queue (Min-Heap) for Dijkstra
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;

    // Get Indices
    int start_i = toIndex(start_x, start_y);
    int end_i = toIndex(end_x, end_y);

    // Start propagation from ROBOT (Start)
    potential[start_i] = 0;
    pq.push({start_i, 0.0f});

    // 3. Define 8-Connected Offsets (Cardinals + Diagonals)
    // Order: Up, Right, Down, Left, Up-Right, Down-Right, Down-Left, Up-Left
    int offsets[8];
    offsets[0] = -nx_;      // Up
    offsets[1] = 1;         // Right
    offsets[2] = nx_;       // Down
    offsets[3] = -1;        // Left
    offsets[4] = -nx_ + 1;  // Up-Right
    offsets[5] = nx_ + 1;   // Down-Right
    offsets[6] = nx_ - 1;   // Down-Left
    offsets[7] = -nx_ - 1;  // Up-Left

    // Costs: 1.0 for Cardinals, 1.414 (sqrt(2)) for Diagonals
    float move_cost[8] = {1.0f, 1.0f, 1.0f, 1.0f, 1.41421f, 1.41421f, 1.41421f, 1.41421f};

    int cycle = 0;
    while (!pq.empty() && cycle < cycles) {
        // Pop the node with the LOWEST accumulated cost
        Node current = pq.top();
        pq.pop();
        
        int curr_i = current.index;

        // --- EARLY EXIT: STOP AT GOAL ---
        if (curr_i == end_i) {
            return true;
        }

        // Check if we found a better path to this node already (Lazy Deletion)
        if (current.cost > potential[curr_i]) continue;

        // Process 8 Neighbors
        for (int i = 0; i < 8; i++) {
            int next_i = curr_i + offsets[i];

            // --- BOUNDARY CHECKS ---
            if (next_i < 0 || next_i >= size) continue;

            // Horizontal Wrap Check
            // Ensure we didn't wrap from Right edge to Left edge or vice versa
            int curr_col = curr_i % nx_;
            int next_col = next_i % nx_;
            if (std::abs(next_col - curr_col) > 1) continue;

            // --- LOGIC ---
            
            // 1. Obstacle Check
            if (costs[next_i] >= lethal_cost_) continue;

            // 2. Calculate New Cost
            float new_cost = potential[curr_i] + move_cost[i];

            // 3. Relaxation: If new path is shorter, update and push to PQ
            if (new_cost < potential[next_i]) {
                potential[next_i] = new_cost;
                pq.push({next_i, new_cost});
            }
        }
        cycle++;
    }

    return true;
}

} // namespace global_planner