#ifndef MAPLAYERBASE_H
#define MAPLAYERBASE_H

#include "ros/ros.h"
#include <string>

class Hypermap;

struct FileData
{
    const char *fname;
    uint8_t *data;
    size_t data_size;
};

class MapLayerBase
{
private:
  const char *tfFrameName;

protected:
  Hypermap *parent;
  bool subscribe_mode;

public:
  MapLayerBase(const char *fName, Hypermap *parent = 0, bool subscribe_mode = false) : tfFrameName(fName), parent(parent), subscribe_mode(subscribe_mode) {}

  virtual void setSubscribeMode(bool mode);
  virtual int getIntValue(double xPos, double yPos) = 0;
  virtual std::string getStringValue(double xPos, double yPos) = 0;

  virtual void loadMapData() = 0;

  virtual void getRGBA();

  void test();
};

#endif // MAPLAYERBASE_H
