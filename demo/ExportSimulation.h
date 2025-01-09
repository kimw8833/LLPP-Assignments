#ifndef _export_simulation_h_
#define _export_simulation_h_

#include "Simulation.h"
#include <string>
#include <fstream>

#define HEATMAP_WIDTH 160 * 5
#define HEATMAP_HEIGHT 120 * 5
#define HEATMAP_SKIP 5

class ExportSimulation : public Simulation {
    public:
        ExportSimulation(Ped::Model &model, int maxSteps,
                std::string outputFilename);
        ExportSimulation() = delete;
        ~ExportSimulation();

        void runSimulation();
    protected:
        std::string outputFilename;
        std::ofstream file;

        void serialize();
};
#endif
