#ifndef SEMANTICLAYER_H
#define SEMANTICLAYER_H

#include "ros/ros.h"
#include "maplayerbase.h"
#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <vector>
#include <string>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

class SemanticLayer : public MapLayerBase
{
  //typedef std::vector< std::pair<std::string, double> > semanticObject;

  typedef bg::model::point<double, 2, bg::cs::cartesian> point;
  typedef bg::model::box<point> box;
  typedef bg::model::polygon<point> polygon;
  typedef std::pair<box, size_t> value;

  struct SemanticObject
  {
    std::string name;
    polygon shape;
  };

  bgi::rtree< value, bgi::rstar<16> > rtree;
  std::vector<SemanticObject> objectList;

public:
  SemanticLayer() : MapLayerBase("map") {}

  int getIntValue(double xPos, double yPos);
  std::string getStringValue(double xPos, double yPos);

  std::vector<std::string> getObjectsInRange(double xmin, double ymin, double xmax, double ymax)
  {
      return getObjectsInRange(point(xmin, ymin), point(xmax, ymax));
  }

  std::vector<std::string> getObjectsInRange(const point &minCorner, const point &maxCorner)
  {
      box query_box(minCorner, maxCorner);
      std::vector<value> result_obj;
      std::vector<std::string> result;
      rtree.query(bgi::intersects(query_box), std::back_inserter(result_obj));
      for (value val : result_obj)
      {
          SemanticObject foundObject = objectList[val.second];
          if (bg::intersects(query_box, foundObject.shape))
              result.push_back(foundObject.name);
      }
      return result;
  }

  void addExampleObject()
  {
      polygon testPoly;
      bg::append(testPoly.outer(), point(0.0, 0.0));
      bg::append(testPoly.outer(), point(0.0, 1.0));
      bg::append(testPoly.outer(), point(1.0, 0.0));
      box bb = bg::return_envelope<box>(testPoly);
      SemanticObject obj = {"TestTriangle", testPoly};
      objectList.push_back(obj);
      rtree.insert(std::make_pair(bb, objectList.size() - 1));
  }

  void printQuery()
  {
      box query_box(point(0.5, 0.51), point(5, 5));
      polygon testPoly;
      bg::append(testPoly.outer(), point(0.0, 0.0));
      bg::append(testPoly.outer(), point(0.0, 1.0));
      bg::append(testPoly.outer(), point(1.0, 0.0));
      std::vector<value> result_s;
      rtree.query(bgi::intersects(query_box), std::back_inserter(result_s));
      ROS_INFO("Printing found objects");
      for (value val : result_s)
      {
          SemanticObject foundObject = objectList[val.second];
          bool intersects = bg::intersects(query_box, foundObject.shape);
          ROS_INFO("Found Object: %s, Intersects: %d\n", foundObject.name.c_str(), intersects);
      }
  }
};

#endif // SEMANTICLAYER_H
