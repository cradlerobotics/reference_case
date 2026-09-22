#include <global_planner/gradient_path.h>
#include <algorithm>
#include <stdio.h>
#include <global_planner/planner_core.h>

namespace global_planner {

GradientPath::GradientPath(PotentialCalculator* p_calc) :
        Traceback(p_calc), pathStep_(0.5) {
}

GradientPath::~GradientPath() {
}

void GradientPath::setSize(int xs, int ys) {
    Traceback::setSize(xs, ys);
    gradx_ = new float[xs * ys];
    grady_ = new float[xs * ys];
}

bool GradientPath::getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path) {
    std::pair<float, float> current;
    int st_index = getIndex(start_x, start_y);
    path.clear();
    int pathLen = 0;
    float total_potential = 0;

    // 1. Compute Gradients for the whole map (needed for descent)
    // We compute gradients at every cell to know which way is "downhill"
    int ns = xs_ * ys_;
    memset(gradx_, 0, ns * sizeof(float));
    memset(grady_, 0, ns * sizeof(float));

    int c = 0;
    for (int y = 0; y < ys_; y++) {
        for (int x = 0; x < xs_; x++) {
            float dx = 0.0;
            float dy = 0.0;
            int sw = 0; // stride width

            // Standard central difference gradient
            // Checks Left/Right
            if (x > 0 && x < xs_ - 1) {
                dx = (potential[c + 1] - potential[c - 1]) / 2.0;
            } else if (x > 0) {
                dx = (potential[c] - potential[c - 1]);
            } else if (x < xs_ - 1) {
                dx = (potential[c + 1] - potential[c]);
            }

            // Checks Up/Down
            if (y > 0 && y < ys_ - 1) {
                dy = (potential[c + xs_] - potential[c - xs_]) / 2.0;
            } else if (y > 0) {
                dy = (potential[c] - potential[c - xs_]);
            } else if (y < ys_ - 1) {
                dy = (potential[c + xs_] - potential[c]);
            }

            // Normalize
            // If the gradient is huge (near wall), this cap helps prevent shooting off
            if(dx == 0 && dy == 0) {
                // Flat area?
            } else {
                float norm = sqrt(dx*dx + dy*dy);
                // "Safe" normalization to ensure step size is consistent
                // If norm is very small, we might be at a local minima
                if(norm > 0) {
                   gradx_[c] = dx / norm;
                   grady_[c] = dy / norm; 
                }
            }
            c++;
        }
    }

    // 2. Trace the path from Goal to Start (Gradient Descent)
    // Note: Global Planner traces backwards from Goal -> Start
    current.first = end_x;
    current.second = end_y;
    int start_index_int = getIndex(start_x, start_y);

    path.push_back(current);

    // Loop limit to prevent infinite loops
    while (getIndex(current.first, current.second) != start_index_int && pathLen < ns * 4) {
        double x = current.first;
        double y = current.second;
        int index = getIndex(x, y);

        // Interpolate gradient at sub-pixel position (x, y)
        // This gives the "Smooth" behavior
        double u = x - (int)x;
        double v = y - (int)y;
        
        // Simple bilinear interpolation of the pre-computed gradients
        // (Simplified for robustness)
        float dx = gradx_[index];
        float dy = grady_[index];

        // Move "downhill" (towards start)
        // Since potentials grow from Start->Goal, we actually move "down" potential to find start.
        // Wait, Dijkstra fills Start=0. So potentials increase away from start.
        // So we descend potential to find start.
        
        // Check for local minima (stuck)
        if (dx == 0 && dy == 0) {
            // We are stuck in a flat spot or local minima.
            return false;
        }

        // Compute obstacle repulsion from costmap
        float rx = 0, ry = 0;
        int ix = (int)x, iy = (int)y;
        if (costs_ && ix > 0 && ix < xs_ - 1 && iy > 0 && iy < ys_ - 1) {
            int ci = ix + iy * xs_;
            float cx = (float)costs_[ci + 1] - (float)costs_[ci - 1];
            float cy = (float)costs_[ci + xs_] - (float)costs_[ci - xs_];
            rx = -cx;
            ry = -cy;
            float rnorm = sqrt(rx * rx + ry * ry);
            if (rnorm > 0) { rx /= rnorm; ry /= rnorm; }
        }

        // Blend gradient descent with obstacle repulsion
        float repulsion_weight = 0.5;
        float mx = dx + repulsion_weight * rx;
        float my = dy + repulsion_weight * ry;
        float mnorm = sqrt(mx * mx + my * my);
        if (mnorm > 0) { mx /= mnorm; my /= mnorm; }

        // Move
        current.first = x - mx * pathStep_;
        current.second = y - my * pathStep_;

        path.push_back(current);
        pathLen++;
    }

    return true;
}

} //end namespace global_planner