

#ifndef TRIPREADER_HPP
#define TRIPREADER_HPP

#include <vector>
#include "Trip.hpp"
#include "InputReader.hpp"



class TripReader
{
public:
    // readTrips() reads a sequence of trips from the given input,
    // returning them as a vector of Trip structs.
    std::vector<Trip> readTrips(InputReader& in);    
};



#endif // TRIPREADER_HPP

