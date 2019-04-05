#ifndef SEMANTICLAYER_H
#define SEMANTICLAYER_H

#include <vector>
#include <string>
#include <set>
#include <map>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/console.h>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Polygon.h"
#include "hypermap_msgs/SemanticMap.h"
#include "hypermap_msgs/SemanticObject.h"
#include "hypermap_msgs/SemanticMapUpdate.h"
#include "hypermap_msgs/GetSemanticByArea.h"
#include "hypermap_msgs/GetSemanticByString.h"

#include "maplayerbase.h"
#include "boost_geometry_msgs.h"

namespace hypermap
{

class SemanticLayer : public MapLayerBase
{
public:
  //typedef std::vector< std::pair<std::string, double> > semanticObject;

  typedef std::pair<box, size_t> rtree_entry;

  struct SemanticObject
  {
    std::string name;
    polygon shape;
    point position;
    box bounding_box;
    std::vector<std::string> tags;
    std::vector<double> confidence;
  };

private:
  size_t next_index;
  bgi::rtree< rtree_entry, bgi::rstar<16> > objectRtree;
  std::map<size_t, SemanticObject> objectList;
  std::map<std::string, std::set<size_t>> objectMap;
  hypermap_msgs::SemanticMap mapMsg;

  ros::Publisher semmapPub;

  void updateMap(const hypermap_msgs::SemanticMapUpdate::ConstPtr update);

  void clear();
  void rebuildMapMsg();

  void addObject(const SemanticObject &newObject);
  bool updateObject(size_t id, const SemanticObject &newObject);
  bool removeObject(size_t id);

  hypermap_msgs::SemanticObject semanticObjectToMsg(const SemanticObject &obj);
  SemanticObject createSemanticObjFromMessage(const hypermap_msgs::SemanticObject &msg);

public:
  SemanticLayer(Hypermap *parent = 0, const std::string &name = "SemanticLayer", const std::string &tfFrame = "map", bool subscribe_mode = false, bool enable_update = true);

  virtual ~SemanticLayer()
  {
      ROS_INFO("Semantic Layer destroyed");
  }

  virtual int getIntValue(const geometry_msgs::Point &p);
  virtual std::string getStringValue(const geometry_msgs::Point &p);
  virtual std::vector<std::pair<geometry_msgs::Point, int>> getIntValues(const geometry_msgs::Polygon &area);
  virtual std::vector<std::pair<geometry_msgs::Point, std::string>> getStringValues(const geometry_msgs::Polygon &area);
  //std::vector<std::pair<geometry_msgs::Point, std::string>> getStringReps(const geometry_msgs::Polygon &area);
  virtual std::vector<geometry_msgs::Point> getCoords(int rep, geometry_msgs::Polygon::ConstPtr area);
  virtual std::vector<geometry_msgs::Point> getCoords(const std::string &rep, geometry_msgs::Polygon::ConstPtr area);

  std::set<size_t> getObjectsAt(const point &p);
  std::set<size_t> getObjectsInRange(double xmin, double ymin, double xmax, double ymax);
  std::set<size_t> getObjectsInRange(const point &minCorner, const point &maxCorner);
  std::set<size_t> getObjectsInRange(const polygon &pg);

  bool getSemanticByArea(hypermap_msgs::GetSemanticByArea::Request &req, hypermap_msgs::GetSemanticByArea::Response &res);

  std::set<size_t> getObjectsByName(const std::string &name);
  std::set<size_t> getObjectsByNameInRange(const std::string &name, const point &minCorner, const point &maxCorner);
  std::set<size_t> getObjectsByNameInRange(const std::string &name, const polygon &pg);

  void addExampleObject();

  virtual void publishData();
  virtual void loadMapData(const std::string &file_name);
  virtual void saveMapData();
  bool readMapData(std::istream &input);
  bool writeMapData(std::ostream &output);
  //std::string generateMapData();

  void printQuery();
};

}

#endif // SEMANTICLAYER_H
