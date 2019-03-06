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
    const std::string name;
    const std::string tfFrame;

protected:
  Hypermap *parent;
  bool subscribe_mode;
  std::string file_name;

public:
  MapLayerBase(Hypermap *parent, const std::string &name, const std::string &tfFrame, bool subscribe_mode = false)
      : parent(parent), name(name), tfFrame(tfFrame), subscribe_mode(subscribe_mode), file_name(name)
  {}

  virtual ~MapLayerBase()
  {}

  virtual void setSubscribeMode(bool mode);

  virtual int getIntValue(const geometry_msgs::Point &p) = 0;
  virtual std::string getStringValue(const geometry_msgs::Point &p) = 0;
  virtual std::vector<std::pair<geometry_msgs::Point, int>> getIntValues(const geometry_msgs::Polygon &area) = 0;
  virtual std::vector<std::pair<geometry_msgs::Point, std::string>> getStringValues(const geometry_msgs::Polygon &area) = 0;
  virtual std::vector<geometry_msgs::Point> getCoords(int rep, geometry_msgs::Polygon::ConstPtr area) = 0;
  virtual std::vector<geometry_msgs::Point> getCoords(const std::string &rep, geometry_msgs::Polygon::ConstPtr area) = 0;

  std::string getName() {return name;}
  std::string getTfFrame() {return tfFrame;}
  std::string getFileName() {return file_name;}

  //virtual int getIntValue(const geometry_msgs::Polygon &pg) = 0;
  //virtual std::string getStringValue(const geometry_msgs::Polygon &pg) = 0;

  virtual void publishData() {}

  virtual void loadMapData(const std::string &file_name) = 0;
  virtual void saveMapData() = 0;

  virtual void getRGBA();

  void test();
};

}

#endif // MAPLAYERBASE_H
