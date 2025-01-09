#include "QTSimulation.h"
#include <QApplication>

QTSimulation::QTSimulation(Ped::Model &model_, int maxSteps_) : Simulation(model_, maxSteps_) {}

QTSimulation::QTSimulation(Ped::Model &model_, int maxSteps_, MainWindow *window_, QObject *parent) : QObject(parent), Simulation(model_, maxSteps_) {
    window = window_;
}

void QTSimulation::simulateOneStep()
{
    tickCounter++;
    model.tick();
    window->paint();

    if (maxSimulationSteps-- == 0) {
        QApplication::quit();
    }
}

void QTSimulation::runSimulation()
{
	QObject::connect(&movetimer, SIGNAL(timeout()), this, SLOT(simulateOneStep()));
	movetimer.start();
}

#include "QTSimulation.moc"
