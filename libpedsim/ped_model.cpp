//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_model.h"
#include "ped_waypoint.h"
#include "ped_model.h"
#include <iostream>
#include <stack>
#include <algorithm>
#include <omp.h>
#include <thread>

#ifndef NOCDUA
#include "cuda_testkernel.h"
#endif

#include <stdlib.h>

void Ped::Model::setup(std::vector<Ped::Tagent*> agentsInScenario, std::vector<Twaypoint*> destinationsInScenario, IMPLEMENTATION implementation)
{
#ifndef NOCUDA
	// Convenience test: does CUDA work on this machine?
	cuda_test();
#else
    std::cout << "Not compiled for CUDA" << std::endl;
#endif

	// Set 
	agents = std::vector<Ped::Tagent*>(agentsInScenario.begin(), agentsInScenario.end());

	// Set up destinations
	destinations = std::vector<Ped::Twaypoint*>(destinationsInScenario.begin(), destinationsInScenario.end());

	// Sets the chosen implemenation. Standard in the given code is SEQ
	this->implementation = implementation;

	// Set up heatmap (relevant for Assignment 4)
	setupHeatmapSeq();
}
//////////////////
/// Assignment 1
//////////////////

void updateAgentPosition(Ped::Tagent* agent) {
    if(agent) {
        agent->computeNextDesiredPosition();
        agent->setX(agent->getDesiredX());
        agent->setY(agent->getDesiredY());
    }
}

void Ped::Model::tick()
{

    switch (implementation)
    {
        case SEQ:
        { // Sequential update of all agents
            for (int i = 0; i < agents.size(); ++i)
            {
                updateAgentPosition(agents[i]);
            }
        }
        break;

        case OMP:
        { // Parallel update using OpenMP
            #pragma omp parallel for
            for (int i = 0; i < agents.size(); ++i)
            {
                updateAgentPosition(agents[i]);
            }
        }
        break;

        case PTHREAD:
        { // Multi-threaded update using C++ threads
            std::vector<std::thread> threads;

            for (int i = 0; i < 8; ++i) {
                threads.emplace_back([&, i]() {
                    for (int j = i; j < agents.size(); j += 8) {
                        updateAgentPosition(agents[j]);
                    }
                });
            }

            for (auto& t : threads) {
                t.join();
            }
        }
        break;

//////////////////
/// Assignment 2
//////////////////

		case VECTOR:
		{ 
			size_t size     = agents.size();
			size_t i        = 0;
			int remainder   = size % 4;

			// Temporary storage for aligned SIMD writes
			alignas(16) int tempX[4];
			alignas(16) int tempY[4];

			for (; i + 4 <= size; i += 4) 
			{	
				// Compute next desired positions for 4 agents
				agents[i]->computeNextDesiredPosition();
				agents[i + 1]->computeNextDesiredPosition();
				agents[i + 2]->computeNextDesiredPosition();
				agents[i + 3]->computeNextDesiredPosition();

				// Load desired positions into SIMD registers
				__m128i desiredX = _mm_set_epi32(
					agents[i + 3]->getDesiredX(),
					agents[i + 2]->getDesiredX(),
					agents[i + 1]->getDesiredX(),
					agents[i]->getDesiredX()
				);

				__m128i desiredY = _mm_set_epi32(
					agents[i + 3]->getDesiredY(),
					agents[i + 2]->getDesiredY(),
					agents[i + 1]->getDesiredY(),
					agents[i]->getDesiredY()
				);

				// Store new positions in a temporary buffer first
				_mm_store_si128((__m128i*) tempX, desiredX);
				_mm_store_si128((__m128i*) tempY, desiredY);

				//Copy values safely to agents
				agents[i]->setX(tempX[0]); agents[i]->setY(tempY[0]);
				agents[i + 1]->setX(tempX[1]); agents[i + 1]->setY(tempY[1]);
				agents[i + 2]->setX(tempX[2]); agents[i + 2]->setY(tempY[2]);
				agents[i + 3]->setX(tempX[3]); agents[i + 3]->setY(tempY[3]);
			}

			// Handle remaining agents (in case of non-multiples of 4)
			for (i = size - remainder; i < size; i++) 
			{
				updateAgentPosition(agents[i]);  // Process remaining agents one-by-one
			}
		}
		break;

    } // end of switch
}

////////////
/// Everything below here relevant for Assignment 3.
/// Don't use this for Assignment 1!
///////////////////////////////////////////////

// Moves the agent to the next desired position. If already taken, it will
// be moved to a location close to it.
void Ped::Model::move(Ped::Tagent *agent)
{
	// Search for neighboring agents
	set<const Ped::Tagent *> neighbors = getNeighbors(agent->getX(), agent->getY(), 2);

	// Retrieve their positions
	std::vector<std::pair<int, int> > takenPositions;
	for (std::set<const Ped::Tagent*>::iterator neighborIt = neighbors.begin(); neighborIt != neighbors.end(); ++neighborIt) {
		std::pair<int, int> position((*neighborIt)->getX(), (*neighborIt)->getY());
		takenPositions.push_back(position);
	}

	// Compute the three alternative positions that would bring the agent
	// closer to his desiredPosition, starting with the desiredPosition itself
	std::vector<std::pair<int, int> > prioritizedAlternatives;
	std::pair<int, int> pDesired(agent->getDesiredX(), agent->getDesiredY());
	prioritizedAlternatives.push_back(pDesired);

	int diffX = pDesired.first - agent->getX();
	int diffY = pDesired.second - agent->getY();
	std::pair<int, int> p1, p2;
	if (diffX == 0 || diffY == 0)
	{
		// Agent wants to walk straight to North, South, West or East
		p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
		p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
	}
	else {
		// Agent wants to walk diagonally
		p1 = std::make_pair(pDesired.first, agent->getY());
		p2 = std::make_pair(agent->getX(), pDesired.second);
	}
	prioritizedAlternatives.push_back(p1);
	prioritizedAlternatives.push_back(p2);

	// Find the first empty alternative position
	for (std::vector<pair<int, int> >::iterator it = prioritizedAlternatives.begin(); it != prioritizedAlternatives.end(); ++it) {

		// If the current position is not yet taken by any neighbor
		if (std::find(takenPositions.begin(), takenPositions.end(), *it) == takenPositions.end()) {

			// Set the agent's position 
			agent->setX((*it).first);
			agent->setY((*it).second);

			break;
		}
	}
}

/// Returns the list of neighbors within dist of the point x/y. This
/// can be the position of an agent, but it is not limited to this.
/// \date    2012-01-29
/// \return  The list of neighbors
/// \param   x the x coordinate
/// \param   y the y coordinate
/// \param   dist the distance around x/y that will be searched for agents (search field is a square in the current implementation)
set<const Ped::Tagent*> Ped::Model::getNeighbors(int x, int y, int dist) const {

	// create the output list
	// ( It would be better to include only the agents close by, but this programmer is lazy.)	
	return set<const Ped::Tagent*>(agents.begin(), agents.end());
}

void Ped::Model::cleanup() {
	// Nothing to do here right now. 
}

Ped::Model::~Model()
{
	std::for_each(agents.begin(), agents.end(), [](Ped::Tagent *agent){delete agent;});
	std::for_each(destinations.begin(), destinations.end(), [](Ped::Twaypoint *destination){delete destination; });
}
