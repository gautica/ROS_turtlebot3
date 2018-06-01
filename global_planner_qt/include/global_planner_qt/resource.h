#ifndef RESOURCE_H
#define RESOURCE_H

#include <string>
#include <stdlib.h>
#include <vector>

namespace global_planner {
class Resource
{
public:
    struct Resource_t
    {
        std::string name;
        float x;
        float y;
        float distance;
    };
public:
    static std::string current_resource;

public:
    Resource();
};
}
#endif // RESOURCE_H
