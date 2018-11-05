

#ifndef ROADMAPWRITER_HPP
#define ROADMAPWRITER_HPP

#include <ostream>
#include "RoadMap.hpp"



class RoadMapWriter
{
public:
    // writeRoadMap() writes a RoadMap to the given output stream (e.g.,
    // you could pass std::cout to write it to the console) in a format
    // that's designed to assist in debugging.
    void writeRoadMap(std::ostream& out, const RoadMap& roadMap);
};



#endif // ROADMAPWRITER_HPP

