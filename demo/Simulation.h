#ifndef _abs_simulation_h_
#define _abs_simulation_h_

#include "ped_model.h"

class Simulation {
    public:
        Simulation(Ped::Model &model_, int maxSteps)
            : model(model_), maxSimulationSteps(maxSteps), tickCounter(0)
            {}
        Simulation() = delete;
        ~Simulation() {}

        virtual int getTickCount() const { return tickCounter; };
        virtual void runSimulation() = 0;
    protected:
        Ped::Model &model;
        int maxSimulationSteps;
        int tickCounter;
};

#endif
