#ifndef _WAVEFRONT_H
#define _WAVEFRONT_H

#include <global_planner/expander.h>
#include <queue>

namespace global_planner {

class WavefrontExpansion : public Expander {
    public:
        WavefrontExpansion(PotentialCalculator* p_calc, int nx, int ny);
        bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                 int cycles, float* potential);
};

} // namespace global_planner
#endif