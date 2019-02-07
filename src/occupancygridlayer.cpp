#include "occupancygridlayer.h"
#include "hypermap.h"

OccupancyGridLayer::OccupancyGridLayer() : MapLayerBase("map")
{
    mapPub = parent->nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);
    mapMetaPub = parent->nh.advertise<nav_msgs::MapMetaData>("/map_metadata", 1);

    if (subscribe_mode)
    {
        mapSub = parent->nh.subscribe("/map", 100, &OccupancyGridLayer::updateMap, this);
        mapMetaSub = parent->nh.subscribe("/map_metadata", 100, &OccupancyGridLayer::updateMapMeta, this);
    }
}

int OccupancyGridLayer::getIntValue(double xPos, double yPos)
{
    return 0;
}

std::string OccupancyGridLayer::getStringValue(double xPos, double yPos)
{
    return "";
}

void OccupancyGridLayer::setSubscribeMode(bool mode)
{
    if (subscribe_mode == true && mode == false)
    {
        mapSub.shutdown();
        mapMetaSub.shutdown();
    }
    else if (subscribe_mode = false && mode == true)
    {
        mapSub = parent->nh.subscribe("/map", 100, &OccupancyGridLayer::updateMap, this);
        mapMetaSub = parent->nh.subscribe("/map_metadata", 100, &OccupancyGridLayer::updateMapMeta, this);
    }
    subscribe_mode = mode;
}

void OccupancyGridLayer::loadMapData()
{
    if (parent != 0)
    {
        parent->testZip();
    }
}

void OccupancyGridLayer::publishData()
{
}
