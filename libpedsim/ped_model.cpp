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
#include <stdlib.h>
#include <iostream>
#include <stack>
#include <algorithm>
#include <omp.h>
#include <thread>
#include <immintrin.h>  // For SIMD intrinsics (AVX, SSE)

#ifndef NOCDUA
#include "cuda_testkernel.h"
#endif

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

	// A2 SIMD
    
	if (implementation == VECTOR)
    {
        // Save the number of agents
        numAgents   = agents.size();
        
        //
        // Allocate aligned memory (Linux/macOS: posix_memalign)
        //
        // Choose the no. of bytes
        size_t fourAgents   = 16; // SSE 128bits
        size_t eightAgents  = 32; // AVX
    
        if (numAgents > 0) {

            if (posix_memalign((void**)&xPos,       fourAgents, numAgents   * sizeof(float)) != 0 ||
                posix_memalign((void**)&yPos,       fourAgents, numAgents   * sizeof(float)) != 0 ||
                posix_memalign((void**)&xDestPos,   fourAgents, numAgents   * sizeof(float)) != 0 ||
                posix_memalign((void**)&yDestPos,   fourAgents, numAgents   * sizeof(float)) != 0 ||
                posix_memalign((void**)&destR,      fourAgents, numAgents   * sizeof(float)) != 0 ) {
                exit(EXIT_FAILURE);
            }
        }

        for (size_t i = 0; i < numAgents; i++) {

            xPos[i]     = agents[i]->getX();
            yPos[i]     = agents[i]->getY();

            agents[i]->destInit();
            xDestPos[i] = agents[i]->getDestX();
            yDestPos[i] = agents[i]->getDestY();
            destR[i]    = agents[i]->getRadius();
            
        }
    }
}

void Ped::Model::updateAgentPosition(Ped::Tagent* agent) {
    if(agent) {
        agent->computeNextDesiredPosition();
        move(agent);
        //agent->setX(agent->getDesiredX());
        //agent->setY(agent->getDesiredY());
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

            for (int i = 0; i < 4; ++i) {
                threads.emplace_back([&, i]() {
                    for (int j = i; j < agents.size(); j += 4) {
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

		case VECTOR: // SSE-based processing 
        { 
            size_t i = 0;
            for (; i + 4 <= numAgents; i += 4) {         

                // Load data into SIMD registers
                __m128 agent_xs = _mm_load_ps(&xPos[i]); 
                __m128 agent_ys = _mm_load_ps(&yPos[i]); 
                __m128 dest_x   = _mm_load_ps(&xDestPos[i]); 
                __m128 dest_y   = _mm_load_ps(&yDestPos[i]); 
                __m128 radius   = _mm_load_ps(&destR[i]);

                // Get next destination function med simd
       		    __m128 diffx    = _mm_sub_ps(dest_x, agent_xs); 
        	    __m128 diffy    = _mm_sub_ps(dest_y, agent_ys); 
                 
                diffx           = _mm_mul_ps(diffx, diffx);  
                diffy           = _mm_mul_ps(diffy, diffy); 

                __m128 sum      = _mm_add_ps(diffx, diffy);  
                __m128 length   = _mm_sqrt_ps(sum); // Euclidean distance

                // Prevent division by zero
                __m128 epsilon  = _mm_set1_ps(1e-6f);
                length          = _mm_add_ps(length, epsilon);
                
                // Check if agent has reached it's destination 
        	    __m128 agentReachedDestination = _mm_cmplt_ps(length, radius); // 0 = not reached, 1 = reached
                
        	    alignas(16) int results[4];
        	    _mm_store_ps(reinterpret_cast<float*>(results), agentReachedDestination);
                
    	        // Update destination this is still done sequentially
                //#pragma omp simd
                for (int j = 0; j < 4; j++) {

                    if (results[j] != 0) { 
                        
                        if (i + j < agents.size()){
                            
                            agents[i+j]->updateDestinationList();
                            agents[i+j]->destInit();
                            xDestPos[i+j] = (float)agents[i + j]->getDestX();     
                            yDestPos[i+j] = (float)agents[i + j]->getDestY();
                            destR[i+j]    = (float)agents[i + j]->getRadius();
                        }  
                    } 
                }

                // Compute next desired position function with simd
                diffx = _mm_sub_ps(dest_x, agent_xs); 
                diffy = _mm_sub_ps(dest_y, agent_ys); 

                __m128 xmul = _mm_mul_ps(diffx, diffx);  
                __m128 ymul = _mm_mul_ps(diffy, diffy); 
                sum         = _mm_add_ps(xmul, ymul);  
                length      = _mm_sqrt_ps(sum); 

                __m128 diffX_div_len = _mm_div_ps(diffx, length);
                __m128 diffY_div_len = _mm_div_ps(diffy, length);

                __m128 add_x_diffx = _mm_add_ps(agent_xs, diffX_div_len); 
                __m128 add_y_diffy = _mm_add_ps(agent_ys, diffY_div_len);
                
                __m128 p5 = _mm_set1_ps(0.5);

                add_x_diffx= _mm_add_ps(add_x_diffx, p5);
                add_y_diffy= _mm_add_ps(add_y_diffy, p5);

                __m128 desiredPosX = _mm_floor_ps(add_x_diffx);
                __m128 desiredPosY = _mm_floor_ps(add_y_diffy);

                // Store
                _mm_store_ps(&xPos[i], desiredPosX);
                _mm_store_ps(&yPos[i], desiredPosY);

            }
            
            //
            // Last Seq. parts
            //

            // Copy updated positions back to agents
            //#pragma omp simd
            for (size_t j = 0; j < i; j++) {
                
                agents[j]->setX(xPos[j]);
                agents[j]->setY(yPos[j]);
            }
            
            // Handle the remaining agents
            
            for (; i < numAgents; i++) {
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

void Ped::Model::move(Ped::Tagent *agent){

    int total_regions = 4;
    int total_x_values = 154; 


    int region_size = total_x_values/total_regions;

    int region; 
    for (int i = region_size; i < total_x_values; i += region_size)
    {
        for(int j = 1; j < total_regions; j++ )
        {
            //printf("i: %d \n", i);
            if (i == 38)
            {
                region = 1;
                //printf(" region %d\n",j);
                break; 
            }
            if (i == 76)
            {
                region = 2;
                //printf(" region 2");
                break; 
            }
            if (i == 114)
            {
                region = 3;
                //printf(" region 3");
                break; 
            }
            if (i == 152)
            {
                region = 4;
                //printf(" region 4");
                break; 
            }
        }

    }

    set<const Ped::Tagent *> neighbors = getNeighbors(agent->getX(), agent->getY(), region_size, region); 

    std::vector<std::pair<int, int> > takenPositions;
	for (std::set<const Ped::Tagent*>::iterator neighborIt = neighbors.begin(); neighborIt != neighbors.end(); ++neighborIt) {
		std::pair<int, int> position((*neighborIt)->getX(), (*neighborIt)->getY());
		takenPositions.push_back(position);
	}

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

            //printf("setting position");
			// Set the agent's position 
			agent->setX((*it).first);
			agent->setY((*it).second);

			break;
		}
	}
    
    

}

set<const Ped::Tagent*> Ped::Model::getNeighbors(int x, int y, int region_size, int region) const {

    set<const Ped::Tagent*> regionagents; 
	for (int i = 0; i < agents.size(); i ++)
    {
        if (agents[i]->getX() < region_size * region)
        {
            //regionagents.push_back(agents[i]);
            //printf("new agent added %d \n",agents[i]->getX());
            regionagents.insert(agents[i]);
        }

        
        
    }
    return regionagents;
}

// Moves the agent to the next desired position. If already taken, it will
// be moved to a location close to it.
/*void Ped::Model::move(Ped::Tagent *agent)
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
}*/

void Ped::Model::cleanup() {
	// Nothing to do here right now. 
}

Ped::Model::~Model()
{
	std::for_each(agents.begin(), agents.end(), [](Ped::Tagent *agent){delete agent;});
	std::for_each(destinations.begin(), destinations.end(), [](Ped::Twaypoint *destination){delete destination; });

    // A2

    if (xPos) {
        //std::cout << "Freeing xPos in destructor: " << (void*)xPos << std::endl;
        free(xPos);
        xPos = nullptr;
    }
    if (yPos) {
        //std::cout << "Freeing yPos in destructor: " << (void*)yPos << std::endl;
        free(yPos);
        yPos = nullptr;
    }
    if (xDestPos) {
        //std::cout << "Freeing xDestPos in destructor: " << (void*)xDestPos << std::endl;
        free(xDestPos);
        xDestPos = nullptr;
    }
    if (yDestPos) {
        //std::cout << "Freeing yDestPos in destructor: " << (void*)yDestPos << std::endl;
        free(yDestPos);
        yDestPos = nullptr;
    }
    if (destR) {
        //std::cout << "Freeing destR in destructor: " << (void*)destR << std::endl;
        free(destR);
        destR = nullptr;
    }
}
