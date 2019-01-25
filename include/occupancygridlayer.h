#ifndef OCCUPANCYGRIDLAYER_H
#define OCCUPANCYGRIDLAYER_H

#include "ros/ros.h"
#include "maplayerbase.h"


class OccupancyGridLayer : public MapLayerBase
{
public:
  OccupancyGridLayer() : MapLayerBase("map") {}

  int getIntValue(double xPos, double yPos);
  std::string getStringValue(double xPos, double yPos);
};

#endif // OCCUPANCYGRIDLAYER_H
