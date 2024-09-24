#include "gcs.h"

BaseGCS::BaseGCS(/* args */)
{
}

BaseGCS::BaseGCS(const std::vector<drake::geometry::optimization::HPolyhedron> &regions)
{
    this->regions = regions;
    for (size_t i = 0; i < regions.size(); i++)
    {
        names.push_back("v" + std::to_string(i));
    }
    dimension = regions[0].ambient_dimension();
    for(auto &r : this->regions)
    {
        assert(r.ambient_dimension() == dimension);
    }
    
}

BaseGCS::~BaseGCS()
{
}

LinearGCS::LinearGCS(/* args */)
{
}

LinearGCS::LinearGCS(const std::vector<drake::geometry::optimization::HPolyhedron> regions, const std::vector<std::pair<int, int>> &edges = std::ref({})):BaseGCS(regions)
{

}

LinearGCS::~LinearGCS()
{
}

