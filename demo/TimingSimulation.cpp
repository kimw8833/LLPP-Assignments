#include "TimingSimulation.h"

using namespace std;

TimingSimulation::TimingSimulation(Ped::Model &model_, int maxSteps) : Simulation(model_, maxSteps) 
{
}

void TimingSimulation::runSimulation()
{
    for (int i = 0; i < maxSimulationSteps; i++) {
        tickCounter++;
        model.tick();
    }
}
