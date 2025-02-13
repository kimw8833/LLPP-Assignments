//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
// Adapted for Low Level Parallel Programming 2017
//
// TAgent represents an agent in the scenario. Each
// agent has a position (x,y) and a number of destinations
// it wants to visit (waypoints). The desired next position
// represents the position it would like to visit next as it
// will bring it closer to its destination.
// Note: the agent will not move by itself, but the movement
// is handled in ped_model.cpp. 
//

#ifndef _ped_agent_h_
#define _ped_agent_h_ 1

#include <vector>
#include <deque>
#include "ped_waypoint.h"

using namespace std;

namespace Ped {
	class Twaypoint;

	class Tagent {
	public:
		Tagent(int posX, int posY);
		Tagent(double posX, double posY);

		// Returns the coordinates of the desired position
		int getDesiredX() const { return desiredPositionX; }
		int getDesiredY() const { return desiredPositionY; }

		// Sets the agent's position
		void setX(int newX) { x = newX; }
		void setY(int newY) { y = newY; }

		// Update the position according to get closer
		// to the current destination
		void computeNextDesiredPosition();

		// Position of agent defined by x and y
		int getX() const { return x; };
		int getY() const { return y; };

		// Adds a new waypoint to reach for this agent
		void addWaypoint(Twaypoint* wp);

		//simd addition
		void updateDestinationList(); 

		void changeDesiredDestination(int desiredx, int desiredy); 

		int getDestX() const {return destX; };
		int getDestY() const {return destY; };
		int getRadius() const {return radius; };

	private:
		Tagent() {};

		// The agent's current position
		int x;
		int y;

		// The agent's desired next position
		int desiredPositionX;
		int desiredPositionY;

		// The current destination (may require several steps to reach)
		Twaypoint* destination;

		// The last destination
		Twaypoint* lastDestination;

		int destX;
		int destY;
		int radius; 

		void initDestination(){
			if(destination != NULL)
			{
				destX = (int)destination->getx();
				destY = (int)destination->gety();
				radius = (int)destination->getr(); 
			}
			else{
				destX = 0; 
				destY = 0; 
				radius = 0; 
			}
		}

		// The queue of all destinations that this agent still has to visit
		deque<Twaypoint*> waypoints;

		// Internal init function 
		void init(int posX, int posY);

		// Returns the next destination to visit
		Twaypoint* getNextDestination();
	};
}

#endif