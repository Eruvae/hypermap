#ifndef MAPLAYERBASE_H
#define MAPLAYERBASE_H

#include "ros/ros.h"
#include <string>

class Hypermap;

class MapLayerBase
{
private:
  const char *tfFrameName;

protected:
  Hypermap *parent;

public:
  MapLayerBase(const char *fName, Hypermap *parent = 0) : tfFrameName(fName), parent(parent) {}

  virtual int getIntValue(double xPos, double yPos) = 0;
  virtual std::string getStringValue(double xPos, double yPos) = 0;

  virtual void loadMapData() = 0;

  void test();
};

#endif // MAPLAYERBASE_H
