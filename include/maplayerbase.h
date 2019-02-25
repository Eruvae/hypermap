#ifndef MAPLAYERBASE_H
#define MAPLAYERBASE_H

#include <string>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>

namespace hypermap
{

class Hypermap;

class MapLayerBase
{
private:
  const char *tfFrameName;

protected:
  Hypermap *parent;
  bool subscribe_mode;
  std::string file_name;

public:
  MapLayerBase(const char *fName, Hypermap *parent = 0, bool subscribe_mode = false) : tfFrameName(fName), parent(parent), subscribe_mode(subscribe_mode) {}

  virtual void setSubscribeMode(bool mode);
  virtual int getIntValue(const geometry_msgs::Point &p) = 0;
  virtual std::string getStringValue(const geometry_msgs::Point &p) = 0;

  //virtual int getIntValue(const geometry_msgs::Polygon &pg) = 0;
  //virtual std::string getStringValue(const geometry_msgs::Polygon &pg) = 0;

  virtual void loadMapData(const std::string &file_name) = 0;
  virtual void saveMapData() = 0;

  virtual void getRGBA();

  void test();
};

}

#endif // MAPLAYERBASE_H
