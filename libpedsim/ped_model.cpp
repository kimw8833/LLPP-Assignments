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
#include <immintrin.h>
#include <xmmintrin.h>

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
        case VECTOR:
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

		case SEQ:
		{ 
			
	
    	int remainder = agents.size() % 4; 

    	//divide agents into simd agents = vectors
    	for(int i = 0; i < agents.size()-remainder; i += 4)
    	{
        	alignas(16) float agent_x_array[4], agent_y_array[4], dest_x_array[4], dest_y_array[4], radius_array[4];
			
        

        	// Load the agent data into SIMD registers
        	for (int j = 0; j < 4 && (i + j) < agents.size()-remainder; ++j) {
		
				agent_x_array[j] = (float)agents[i + j]->getX();
				agent_y_array[j] = (float)agents[i + j]->getY();
				dest_x_array[j] = (float)agents[i + j]->getDestX();
				dest_y_array[j] = (float)agents[i + j]->getDestY();
				radius_array[j] = (float)agents[i + j]->getRadius();
			
        	}
        
        	// Load data into SIMD registers
        	__m128 agent_xs = _mm_load_ps(agent_x_array);
    		__m128 agent_ys = _mm_load_ps(agent_y_array);
    		__m128 dest_x = _mm_load_ps(dest_x_array);
    		__m128 dest_y = _mm_load_ps(dest_y_array);
    		__m128 radius = _mm_load_ps(radius_array);

        	//get next destination function med simd
       		__m128 diffx = _mm_sub_ps(dest_x, agent_xs); 
        	__m128 diffy = _mm_sub_ps(dest_y, agent_ys); 

        	diffx = _mm_mul_ps(diffx, diffx);  
        	diffy = _mm_mul_ps(diffy, diffy); 
        	__m128 sum = _mm_add_ps(diffx, diffy);  
        	__m128 length = _mm_sqrt_ps(sum);  // Euclidean distance
			// Prevent division by zero
			__m128 epsilon = _mm_set1_ps(1e-6f);
			length = _mm_add_ps(length, epsilon);

        	// check if agent has reached it's destination 
			//puts 0 for not reached one for reached
        	__m128 agentReachedDestination = _mm_cmplt_ps(length, radius);

        	alignas(16) int results[4];
        	_mm_store_ps(reinterpret_cast<float*>(results), agentReachedDestination);

			//update destination this is still done sequentially ASK about that 
        	for (int j = 0; j < 4; j++) {
            	if (results[j] != 0) { 
                	if (i + j < agents.size()){
						agents[i + j]->updateDestinationList();
					}  
            	} 
        	}

			for (int j = 0; j < 4 && (i + j) < agents.size()-remainder; ++j) {
		
				dest_x_array[j] = (float)agents[i + j]->getDestX();
				dest_y_array[j] = (float)agents[i + j]->getDestY();
				radius_array[j] = (float)agents[i + j]->getRadius();
			
        	}

			dest_x = _mm_load_ps(dest_x_array);
    		dest_y = _mm_load_ps(dest_y_array);
    		radius = _mm_load_ps(radius_array);
		

        	// compute next desired position function with simd
			diffx = _mm_sub_ps(dest_x, agent_xs); 
        	diffy = _mm_sub_ps(dest_y, agent_ys); 

        	diffx = _mm_mul_ps(diffx, diffx);  
        	diffy = _mm_mul_ps(diffy, diffy); 
        	sum = _mm_add_ps(diffx, diffy);  
        	length = _mm_sqrt_ps(sum); 

			__m128 add_x_diffx = _mm_add_ps(agent_xs, dest_x); 
			__m128 add_y_diffy = _mm_add_ps(agent_ys, dest_y);
        	__m128 diffX_div_len = _mm_div_ps(add_x_diffx, length);
        	__m128 diffY_div_len = _mm_div_ps(add_y_diffy, length);

        	__m128 desiredPosX = _mm_add_ps(agent_xs, diffX_div_len);
        	__m128 desiredPosY = _mm_add_ps(agent_ys, diffY_div_len);

        	__m128i intPosX = _mm_cvtps_epi32(desiredPosX);
    		__m128i intPosY = _mm_cvtps_epi32(desiredPosY);
        	// Store the rounded values into arrays
        	alignas(16) int posX_values[4], posY_values[4];
    		_mm_store_si128(reinterpret_cast<__m128i*>(posX_values), intPosX);
    		_mm_store_si128(reinterpret_cast<__m128i*>(posY_values), intPosY);

        	// Update agent desired positions (still sequentially)
        	for (int j = 0; j < 4; j++) {
				if (i + j < agents.size()) {
					agents[i + j]->changeDesiredDestination(posX_values[j], posY_values[j]);
					//printf("%d \n", posX_values[j]);
					//printf("%d \n", posY_values[j]);
					agents[i + j]->setX(posX_values[j]);
					agents[i + j]->setY(posY_values[j]);
				}
			
        	}

		
    }

	for(int i = agents.size()-remainder; i < agents.size(); i ++)
	{
		updateAgentPosition(agents[i]);
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
