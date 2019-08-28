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
protected:
  const std::string name;
  const std::string tfFrame;
  Hypermap *parent;
  bool subscribe_mode;
  bool enable_update;
  bool publish_global_topics;
  std::string file_name;
  std::string subscribe_topic;

public:
  MapLayerBase(Hypermap *parent, const std::string &name, const std::string &tfFrame, bool subscribe_mode = false, bool enable_update = true,
               bool publish_global_topics = false, const std::string &subscribe_topic = "")
      : parent(parent), name(name), tfFrame(tfFrame), subscribe_mode(subscribe_mode), enable_update(enable_update), publish_global_topics(publish_global_topics), file_name(name), subscribe_topic(subscribe_topic)
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

  void test();
};

}

#endif // MAPLAYERBASE_H
