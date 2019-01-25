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

  bgi::rtree< value, bgi::rstar<16> > rtree;
  //std::vector<semanticObject> objectList;

public:
  SemanticLayer() : MapLayerBase("map") {}

  int getIntValue(double xPos, double yPos);
  std::string getStringValue(double xPos, double yPos);

  void addExampleObject()
  {
      polygon testPoly;
      bg::append(testPoly.outer(), point(0.0, 0.0));
      bg::append(testPoly.outer(), point(0.0, 1.0));
      bg::append(testPoly.outer(), point(1.0, 0.0));
      box bb = bg::return_envelope<box>(testPoly);
      rtree.insert(std::make_pair(bb, 0));
  }

  void printQuery()
  {
      box query_box(point(1, 1), point(5, 5));
      std::vector<value> result_s;
      rtree.query(bgi::intersects(query_box), std::back_inserter(result_s));
      ROS_INFO("Printing found objects");
      for (value val : result_s)
      {
          ROS_INFO("Found Object: %lu\n", val.second);
      }
  }
};

#endif // SEMANTICLAYER_H
