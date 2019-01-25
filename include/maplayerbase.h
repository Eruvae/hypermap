#ifndef MAPLAYERBASE_H
#define MAPLAYERBASE_H

#include "ros/ros.h"
#include <string>

class MapLayerBase
{
private:
  const char *tfFrameName;
public:
  MapLayerBase(const char *fName) : tfFrameName(fName) {}

  virtual int getIntValue(double xPos, double yPos) = 0;
  virtual std::string getStringValue(double xPos, double yPos) = 0;
};

#endif // MAPLAYERBASE_H
