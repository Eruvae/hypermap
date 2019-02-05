#include "occupancygridlayer.h"
#include "hypermap.h"

/*OccupancyGridLayer::OccupancyGridLayer() : MapLayerBase("map")
{

}*/

int OccupancyGridLayer::getIntValue(double xPos, double yPos)
{
    return 0;
}

std::string OccupancyGridLayer::getStringValue(double xPos, double yPos)
{
    return "";
}

void OccupancyGridLayer::loadMapData()
{
    if (parent != 0)
    {
        parent->testZip();
    }
}
