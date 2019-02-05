#ifndef OCCUPANCYGRIDLAYER_H
#define OCCUPANCYGRIDLAYER_H

#include "ros/ros.h"
#include "maplayerbase.h"


class OccupancyGridLayer : public MapLayerBase
{
public:
  OccupancyGridLayer() : MapLayerBase("map") {}

  virtual int getIntValue(double xPos, double yPos);
  virtual std::string getStringValue(double xPos, double yPos);
  virtual void loadMapData();
};

#endif // OCCUPANCYGRIDLAYER_H
