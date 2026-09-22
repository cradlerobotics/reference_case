#include <global_planner/dijkstra.h>
#include <algorithm>
#include <stdio.h>
#include <queue>
#include <utility>

namespace global_planner {

DijkstraExpansion::DijkstraExpansion(PotentialCalculator* p_calc, int nx, int ny) :
        Expander(p_calc, nx, ny), pending_(NULL), precise_(false) {
    // priority buffers
    buffer1_ = new int[nx * ny];
    buffer2_ = new int[nx * ny];
    buffer3_ = new int[nx * ny];

    priorityIncrement_ = 2 * neutral_cost_;
}

DijkstraExpansion::~DijkstraExpansion() {
    delete[] buffer1_;
    delete[] buffer2_;
    delete[] buffer3_;
    if (pending_)
        delete[] pending_;
}

void DijkstraExpansion::setSize(int nx, int ny) {
    Expander::setSize(nx, ny);
    if (pending_)
        delete[] pending_;

    pending_ = new bool[nx * ny];
    memset(pending_, 0, nx * ny * sizeof(bool));
}

//
// SRAS MODIFIED DIJKSTRA
// Adds a quadratic penalty to non-lethal obstacles to force the path to the center.
//
bool DijkstraExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                           int cycles, float* potential) {
    cells_visited_ = 0;
    // initial setup
    float INF = 1e10;
    std::fill(potential, potential + ns_, INF);
    std::fill(pending_, pending_ + ns_, false);
    memset(buffer1_, 0, ns_ * sizeof(int));
    memset(buffer2_, 0, ns_ * sizeof(int));
    memset(buffer3_, 0, ns_ * sizeof(int));

    int start_i = toIndex(start_x, start_y);
    int end_i = toIndex(end_x, end_y);
    potential[start_i] = 0;

    // Min-heap priority queue: (potential, cell_index)
    typedef std::pair<float, int> PQEntry;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry> > pq;
    pq.push(PQEntry(0.0f, start_i));

    // 8-connectivity: dx, dy, cost_multiplier
    static const int ndx[8] = { 0,  1, 0, -1, 1,  1, -1, -1};
    static const int ndy[8] = {-1,  0, 1,  0, -1,  1,  1, -1};
    static const float ncost[8] = {1.0f, 1.0f, 1.0f, 1.0f, 1.414f, 1.414f, 1.414f, 1.414f};

    while (!pq.empty()) {
        PQEntry top = pq.top();
        pq.pop();
        float curr_pot = top.first;
        int curr_i = top.second;

        // Skip stale entries
        if (curr_pot > potential[curr_i]) continue;

        // Early termination: stop once goal is reached
        if (curr_i == end_i)
            return true;

        cells_visited_++;
        int cx = curr_i % nx_;
        int cy = curr_i / nx_;

        for (int k = 0; k < 8; k++) {
            int nbx = cx + ndx[k];
            int nby = cy + ndy[k];
            if (nbx < 0 || nbx >= nx_ || nby < 0 || nby >= (ns_ / nx_)) continue;

            int next_i = nbx + nby * nx_;

            unsigned char c = costs[next_i];
            if (c >= lethal_cost_) continue;

            float dist = ncost[k] * neutral_cost_;
            float extra_penalty = 0.0;

            if (c > 0) {
                float factor = (float)c / 252.0f;
                extra_penalty = 12.0f * (factor * factor) * neutral_cost_;
            }

            float new_potential = potential[curr_i] + dist + extra_penalty;

            if (new_potential < potential[next_i]) {
                potential[next_i] = new_potential;
                pq.push(PQEntry(new_potential, next_i));
            }
        }
    }
    return true;
}

} //end namespace global_planner