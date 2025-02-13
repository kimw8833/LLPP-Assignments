//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2017
//
#include "ped_agent.h"
#include "ped_waypoint.h"
#include <math.h>

#include <stdlib.h>
#include <cstdio>

Ped::Tagent::Tagent(int posX, int posY) {
	Ped::Tagent::init(posX, posY);
}

Ped::Tagent::Tagent(double posX, double posY) {
	Ped::Tagent::init((int)round(posX), (int)round(posY));
}

void Ped::Tagent::init(int posX, int posY) {
	x = posX;
	y = posY;
	destination = NULL;
	lastDestination = NULL;
}

void Ped::Tagent::computeNextDesiredPosition() {

	destination = getNextDestination();
	if (destination == NULL) {
		// no destination, no need to
		// compute where to move to
		return;
	}

	printf("destination: x %d y %d\n", destination->getx(), destination->gety());

	double diffX = destination->getx() - x;
	double diffY = destination->gety() - y;
	double len = sqrt(diffX * diffX + diffY * diffY);
	desiredPositionX = (int)round(x + diffX / len);
	desiredPositionY = (int)round(y + diffY / len);
	printf("desired position x %d y %d \n", (int)desiredPositionX, (int)desiredPositionY);

}

void Ped::Tagent::addWaypoint(Twaypoint* wp) {
	waypoints.push_back(wp);
}

void Ped::Tagent::updateDestinationList() {
    waypoints.pop_front();
	waypoints.push_back(destination);
	destination = waypoints.front();
}

int Ped::Tagent::getDestX() { return destination->getx(); }
int Ped::Tagent::getDestY() { return destination->gety(); }
int Ped::Tagent::getRadius() { return destination->getr(); }

void Ped::Tagent::destInit() { destination = waypoints.front(); }



void Ped::Tagent::changeDesiredDestination(int desiredx, int desiredy){
	desiredPositionX = desiredx; 
	desiredPositionY = desiredy; 
}

Ped::Twaypoint* Ped::Tagent::getNextDestination() {
	Ped::Twaypoint* nextDestination = NULL;
	bool agentReachedDestination = false;

	if (destination != NULL) {
		// compute if agent reached its current destination
		printf("destinations is not null");
		printf("first destination x: %d y: %d\n", destination->getx(), destination->gety());
		double diffX = destination->getx() - x;
		double diffY = destination->gety() - y;
		double length = sqrt(diffX * diffX + diffY * diffY);
		agentReachedDestination = length < destination->getr();
	}

	if ((agentReachedDestination || destination == NULL) && !waypoints.empty()) {
		// Case 1: agent has reached destination (or has no current destination);
		// get next destination if available
		printf("the destination or agent reached is null");
		waypoints.push_back(destination);
		nextDestination = waypoints.front();
		waypoints.pop_front();
		
	}
	else {
		// Case 2: agent has not yet reached destination, continue to move towards
		// current destination
		nextDestination = destination;
	}

	

	return nextDestination;
}
