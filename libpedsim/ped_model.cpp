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

#include <vector>
#include <unordered_map>
#include <mutex>
#include <queue>
#include <cmath>

///// A3

// Choose the number of threads
const int NUM_THREADS   = 4;

// Constants for the region grid
const int NUM_REGIONS   = 4; // Fixed number of regions (can be more)

// Mutexes for each region (to prevent race conditions)
std::mutex regionLocks[NUM_REGIONS];

// Data structure for regions
std::unordered_map<int, std::vector<Ped::Tagent*>> regionMap;

// Data structure for agent migration
std::queue<Ped::Tagent*> migratingAgents;
std::mutex migrationMutex;

///// A3

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

	///// A2 SIMD
    
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
     
void Ped::Model::updateAgentPosition(Ped::Tagent* agent) 
{
    if(agent) 
    {
        agent->computeNextDesiredPosition();
        // agent->setX(agent->getDesiredX());
        // agent->setY(agent->getDesiredY());
        move(agent);
    }
}

// Assign agents to their respective regions based on coordinates
void Ped::Model::assignAgentsToRegions() 
{
    regionMap.clear();

    // Initialize empty regions
    for (int i = 0; i < NUM_REGIONS; ++i) 
    {
        regionMap[i] = {};  
    }
    // Create the regions with agent by its calculated regionId
    for (auto* agent : agents) 
    {
        int regionId = getRegionId(agent->getX(), agent->getY());
        regionMap[regionId].push_back(agent);
    }
}

// Returns the region ID based on agent coordinates
// For 160x120
int Ped::Model::getRegionId(int x, int y) const{

    int numRegionsPerRow    = 2;  // 2 regions per row → 2x2 grid
    int regionWidth         = 80; // Each region is 80 units wide
    int regionHeight        = 60; // Each region is 60 units tall

    int col = x / regionWidth;
    int row = y / regionHeight;

    // Ensure col and row stay within valid bounds (i.e., (160,120))
    col = std::max(0, std::min(col, numRegionsPerRow - 1));
    row = std::max(0, std::min(row, numRegionsPerRow - 1));

    return row * numRegionsPerRow + col;  // Compute unique region ID 0 1 2 3
}

void Ped::Model::tick()
{
    switch (implementation)
    {

        //
        // Our simple OMP is always here for us 
        //
        // case OMP:
        // { // Parallel update using OpenMP
            
        //     // Set the number of OpenMP threads
        //     omp_set_num_threads(NUM_THREADS);

        //     #pragma omp parallel for
        //     for (int i = 0; i < agents.size(); ++i)
        //     {
        //         updateAgentPosition(agents[i]);
        //     }
        // }
        // break;

        case OMP:
        { 
            omp_set_num_threads(NUM_THREADS);
            assignAgentsToRegions();  // Create the regions

            // Create per-thread migration queues to avoid race conditions
            std::vector<std::queue<Ped::Tagent*>> localMigratingAgents(NUM_THREADS);

            //
            // Move the agents if their next positions belong to the same region, otherwise send them to localMigratingAgents
            //
            #pragma omp parallel 
            {
                int threadID = omp_get_thread_num();

                if (threadID < NUM_REGIONS) 
                {
                    if (!regionMap[threadID].empty()) 
                    {   
                        std::vector<Ped::Tagent*>& agentsInRegion = regionMap[threadID]; // Get a region from regionMap
                        std::vector<Ped::Tagent*> agentsToMigrate; // Prepare for any agent who move outside their region

                        for (auto it = agentsInRegion.begin(); it != agentsInRegion.end(); ++it) 
                        {
                            Ped::Tagent* agent = *it;
                            
                            agent->computeNextDesiredPosition();

                            int newRegionId = getRegionId(agent->getDesiredX(), agent->getDesiredY());

                            if (newRegionId == threadID) // Mean that the agent want to move but still inside the same region
                            {
                                move(agent);  // Move immediately if staying in the same region
                            }
                            else 
                            {
                                agentsToMigrate.push_back(agent);  // Store the agent who want to move outside the old region.
                            }
                        }

                        // Now remove agents that need to migrate (AFTER iteration)
                        for (Ped::Tagent* agent : agentsToMigrate) 
                        {
                            agentsInRegion.erase(std::remove(agentsInRegion.begin(), agentsInRegion.end(), agent), agentsInRegion.end());
                            localMigratingAgents.at(threadID).push(agent);
                        }
                    }
                }
            }

            //
            // Transfer migrating agents from each localMigratingAgents to migratingAgents (global access)
            //
            #pragma omp critical // Prevent race conditions by ensuring that only one thread at a time executes
            {
                for (int i = 0; i < NUM_THREADS; ++i) 
                {
                    while (!localMigratingAgents[i].empty()) 
                    {
                        migratingAgents.push(localMigratingAgents[i].front());
                        localMigratingAgents[i].pop();
                    }
                }
            }

            //
            // From migratingAgents, insert migrating agents into the new region and move them
            //
            #pragma omp parallel
            {
                #pragma omp single nowait
                {
                    while (!migratingAgents.empty()) 
                    {
                        Ped::Tagent* agent;

                        // Get the agent from migratingAgents
                        #pragma omp critical // Again, prevent race conditions by ensuring that only one thread at a time executes
                        {
                            if (!migratingAgents.empty()) 
                            {
                                agent = migratingAgents.front();
                                migratingAgents.pop();
                            }
                        }

                        // Insert them into the new region and move them
                        if (agent)
                        {
                            int newRegionId = getRegionId(agent->getDesiredX(), agent->getDesiredY());

                            if (newRegionId >= 0 && newRegionId < NUM_REGIONS)
                            {
                                // Lock only the new region for safe modification
                                // Since the mutex is locked, only one thread at a time can modify regionMap[newRegionId]
                                std::lock_guard<std::mutex> lock(regionLocks[newRegionId]);
                                regionMap[newRegionId].push_back(agent);

                                // Move the agent after insertion
                                move(agent);
                            }    
                        }
                    }
                }
                #pragma omp taskwait  // Ensures all tasks complete before continuing
            }
        } break;

        case SEQ:
        { // Sequential update of all agents
            for (int i = 0; i < agents.size(); ++i)
            {
                updateAgentPosition(agents[i]);
            }
        }
        break;

        case PTHREAD:
        { // Multi-threaded update using C++ threads
            std::vector<std::thread> threads;

            for (int i = 0; i < NUM_THREADS; ++i) {
                threads.emplace_back([&, i]() {
                    for (int j = i; j < agents.size(); j += NUM_THREADS) {
                        updateAgentPosition(agents[j]);
                    }
                });
            }

            for (auto& t : threads) {
                t.join();
            }
        }
        break;

		case VECTOR: // SSE-based processing 
        { 
            size_t i = 0;
            for (; i + NUM_THREADS <= numAgents; i += NUM_THREADS) {         

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
                for (int j = 0; j < NUM_THREADS; j++) {

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
    
    } 
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
/// \date    2025-02-23
/// \return  The list of neighbors
/// \param   x the x coordinate
/// \param   y the y coordinate
/// \param   dist the distance around x/y that will be searched for agents (search field is a square in the current implementation)

set<const Ped::Tagent*> Ped::Model::getNeighbors(int x, int y, int dist) const // A3
{

    set<const Ped::Tagent*> neighbors;

    for (const auto* agent : agents) {
        int agentX = agent->getX();
        int agentY = agent->getY();

        // Check if the agent is within the square search field 
        if (abs(agentX - x) <= dist && abs(agentY - y) <= dist) {
            neighbors.insert(agent);
        }
    }

    return neighbors;
}

void Ped::Model::cleanup() {
	// Nothing to do here right now. 
}

Ped::Model::~Model()
{
    // std::for_each(agents.begin(), agents.end(), [](Ped::Tagent *agent){delete agent;});
	// std::for_each(destinations.begin(), destinations.end(), [](Ped::Twaypoint *destination){delete destination; });

    ///// A3

    #pragma omp barrier  // Ensure all threads finish before cleanup

    // Delete all agents safely
    for (Ped::Tagent* agent : agents) {
        if (agent) {  // Prevents deleting nullptr
            delete agent;
        }
    }
    agents.clear();  // Prevents use-after-free

    // Delete all destinations safely
    for (Ped::Twaypoint* destination : destinations) {
        if (destination) {  // Prevents deleting nullptr
            delete destination;
        }
    }
    destinations.clear();  // Prevents use-after-free

    #pragma omp barrier // Ensure all deletions complete before clearing regionMap
    regionMap.clear();  // Clear regions after all agents are deleted

    ///// A3

    ///// A2
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
