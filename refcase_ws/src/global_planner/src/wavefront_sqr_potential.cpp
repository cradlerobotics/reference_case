#include <global_planner/wavefront.h>
#include <algorithm>
#include <queue>

namespace global_planner {

WavefrontExpansion::WavefrontExpansion(PotentialCalculator* p_calc, int nx, int ny) :
        Expander(p_calc, nx, ny) {
}

bool WavefrontExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                           int cycles, float* potential) {
    // 1. Initialize all potentials to a high value (Infinity)
    int size = nx_ * ny_;
    std::fill(potential, potential + size, 1e10);

    // 2. Setup Queue (BFS)
    std::queue<int> queue;

    // Convert world coordinates to grid indices
    int start_i = toIndex(start_x, start_y);
    int end_i = toIndex(end_x, end_y); // Calculate Goal Index

    // Start the wave from the ROBOT'S POSITION
    potential[start_i] = 0;
    queue.push(start_i);

    // 3. The Wavefront Loop
    int cycle = 0;
    while (!queue.empty() && cycle < cycles) {
        int curr_i = queue.front();
        queue.pop();

        // --- NEW: EARLY EXIT CONDITION ---
        // If we have just processed the goal cell, stop expanding!
        if (curr_i == end_i) {
            return true; 
        }

        // 4-Connected Neighbors
        // Note: Using raw indices requires careful bounds checking
        int neighbors[4];
        neighbors[0] = curr_i - nx_; // Up
        neighbors[1] = curr_i + 1;   // Right
        neighbors[2] = curr_i + nx_; // Down
        neighbors[3] = curr_i - 1;   // Left

        for (int i = 0; i < 4; i++) {
            int next_i = neighbors[i];

            // Safety Bounds Check (prevent segfaults)
            if (next_i < 0 || next_i >= size) continue;

            // Wrap-around checks (prevent jumping across map edges)
            // If moving Right (index + 1), we shouldn't land on column 0
            if (i == 1 && next_i % nx_ == 0) continue;
            // If moving Left (index - 1), we shouldn't land on the last column
            if (i == 3 && (next_i + 1) % nx_ == 0) continue;

            // Obstacle Check (Lethal cost)
            if (costs[next_i] >= lethal_cost_) continue;

            // Optimization: If we've already visited (potential is not infinite), skip
            if (potential[next_i] < 1e9) continue;

            // Update Potential (Distance from Start)
            potential[next_i] = potential[curr_i] + 1.0;
            queue.push(next_i);
        }
        cycle++;
    }

    return true;
}

} // namespace global_planner