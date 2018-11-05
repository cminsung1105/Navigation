// main.cpp
//
// ICS 46 Spring 2018
// Project #5: Rock and Roll Stops the Traffic
//
// This is the program's main() function, which is the entry point for your
// console user interface.

#include "InputReader.hpp"
#include "TripReader.hpp"
#include "RoadMap.hpp"
#include "RoadMapReader.hpp"
#include <iostream>
#include <vector>

struct Navigation
{
	int vertex;
	std::string street;
	RoadSegment info;
};


int main()
{
	RoadMapReader readroadmap;
	InputReader readinput(std::cin);
	RoadMap roads = readroadmap.readRoadMap(readinput);

	if (!roads.isStronglyConnected())
	{
		std::cout << "DISCONNECTED MAP" << std::endl;
		return 0;
	}

	TripReader readtrips;
	//VECTOR OF TRIP STRUCTS
	std::vector<Trip> trips = readtrips.readTrips(readinput);

	for (std::vector<Trip>::const_iterator it = trips.begin(); it != trips.end(); ++it)
	{
		std::vector<navigation> navi;
		std::map<int,int> paths;

		if (it->metric == TripMetric::Distance)
		{
			paths = roads.findShortestPaths(it->startVertex, [](RoadSegment segment){return segment.miles;});
		}
		else
		{
			paths = roads.findShortestPaths(it->startVertex, [](RoadSegment segment){return segment.milesPerHour;});
		}



	}
    return 0;
}

