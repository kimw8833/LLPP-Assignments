#ifndef _timing_simulation_h_
#define _timing_simulation_h_

#include "Simulation.h"

class TimingSimulation : public Simulation {
    public:
        TimingSimulation(Ped::Model &model, int maxSteps);
        TimingSimulation();
        ~TimingSimulation() {};

        void runSimulation();
};
#endif
