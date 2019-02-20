#ifndef SEMANTICLAYER_H
#define SEMANTICLAYER_H

#include <vector>
#include <string>
#include <set>
#include <map>

#include <ros/ros.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/polygon/voronoi.hpp>
#include <boost/geometry/geometries/adapted/boost_polygon.hpp>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Polygon.h"
#include "hypermap_msgs/SemanticObject.h"
#include "hypermap_msgs/SemanticMapUpdate.h"
#include "hypermap_msgs/GetSemanticByArea.h"
#include "hypermap_msgs/GetSemanticByString.h"

#include "maplayerbase.h"

namespace hypermap
{

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

class SemanticLayer : public MapLayerBase
{
public:
  //typedef std::vector< std::pair<std::string, double> > semanticObject;

  //typedef bg::model::point<double, 2, bg::cs::cartesian> point;
  typedef bg::model::d2::point_xy<double> point;
  typedef bg::model::box<point> box;
  typedef bg::model::polygon<point> polygon;
  typedef bg::model::ring<point> ring;
  typedef std::pair<box, size_t> rtree_entry;

  struct SemanticObject
  {
    std::string name;
    polygon shape;
    box bounding_box;
  };

private:
  size_t next_index;
  bgi::rtree< rtree_entry, bgi::rstar<16> > objectRtree;
  std::map<size_t, SemanticObject> objectList;
  std::map<std::string, std::set<size_t>> objectMap;

  void polygonToTriangles(const ring &pg);

  void updateMap(const hypermap_msgs::SemanticMapUpdate::ConstPtr update);

  void addObject(const SemanticObject &newObject);
  bool updateObject(size_t id, const SemanticObject &newObject);
  bool removeObject(size_t id);

  hypermap_msgs::SemanticObject semanticObjectToMsg(const SemanticObject &obj);
  SemanticObject createSemanicObjFromMessage(const hypermap_msgs::SemanticObject &msg);
  geometry_msgs::Point boostToPointMsg(const point &p);
  geometry_msgs::Point32 boostToPoint32Msg(const point &p);
  geometry_msgs::Polygon boostToPolygonMsg(const polygon &pg);
  point pointMsgToBoost(const geometry_msgs::Point &pm);
  point point32MsgToBoost(const geometry_msgs::Point32 &pm);
  polygon polygonMsgToBoost(const geometry_msgs::Polygon &pgm);

public:
  SemanticLayer(Hypermap *parent = 0) : MapLayerBase("map", parent), next_index(0) {}

  virtual int getIntValue(double xPos, double yPos);
  virtual std::string getStringValue(double xPos, double yPos);
  virtual void loadMapData(const std::string &file_name);

  std::set<size_t> getObjectsAt(const point &p);
  std::set<size_t> getObjectsInRange(double xmin, double ymin, double xmax, double ymax);
  std::set<size_t> getObjectsInRange(const point &minCorner, const point &maxCorner);
  std::set<size_t> getObjectsInRange(const polygon &pg);

  bool getSemanticByArea(hypermap_msgs::GetSemanticByArea::Request &req, hypermap_msgs::GetSemanticByArea::Response &res);

  std::set<size_t> getObjectsByName(const std::string &name);
  std::set<size_t> getObjectsByNameInRange(const std::string &name, const point &minCorner, const point &maxCorner);

  std::vector<std::pair<geometry_msgs::Point, std::string>> getStringReps(const geometry_msgs::Polygon &area);

  void addExampleObject();

  void readMapData(const std::string &data);
  std::string generateMapData();

  void printQuery();
};

}

#endif // SEMANTICLAYER_H
