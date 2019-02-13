#ifndef SEMANTICLAYER_H
#define SEMANTICLAYER_H

#include "ros/ros.h"
#include "maplayerbase.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Polygon.h"
#include "hypermap_msgs/SemanticObject.h"
#include "hypermap_msgs/SemanticMapUpdate.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <boost/polygon/voronoi.hpp>
#include <boost/geometry/geometries/adapted/boost_polygon.hpp>
#include <vector>
#include <string>
#include <set>
#include <map>

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

class SemanticLayer : public MapLayerBase
{
  //typedef std::vector< std::pair<std::string, double> > semanticObject;

  //typedef bg::model::point<double, 2, bg::cs::cartesian> point;
  typedef bg::model::d2::point_xy<double> point;
  typedef bg::model::box<point> box;
  typedef bg::model::polygon<point> polygon;
  typedef bg::model::ring<point> ring;
  typedef std::pair<box, size_t> value;

  struct SemanticObject
  {
    std::string name;
    polygon shape;
    box bounding_box;
  };

  size_t next_index;
  bgi::rtree< value, bgi::rstar<16> > objectRtree;
  std::map<size_t, SemanticObject> objectList;
  std::map<std::string, std::set<size_t>> objectMap;

  void polygonToTriangles(const ring &pg)
  {
      //boost::polygon::voronoi_diagram<double> vd;
      //boost::polygon::construct_voronoi(pg.begin(), pg.end(), &vd);

  }

  void updateMap(const hypermap_msgs::SemanticMapUpdate::ConstPtr update)
  {
      for (const hypermap_msgs::SemanticObject &obj : update->to_add)
      {
          addObject(createSemanicObjFromMessage(obj));
      }
      for (size_t id : update->to_modify)
      {
          updateObject(id, createSemanicObjFromMessage(update->updates[id]));
      }
      for (size_t id : update->to_delete)
      {
          removeObject(id);
      }
  }

  SemanticObject createSemanicObjFromMessage(const hypermap_msgs::SemanticObject &msg)
  {
      SemanticObject obj;
      obj.name = msg.name;
      obj.shape = polygonMsgToBoost(msg.shape);
      obj.bounding_box = bg::return_envelope<box>(obj.shape);
  }

  void addObject(const SemanticObject &newObject)
  {
      objectList[next_index] = newObject;
      objectMap[newObject.name].insert(next_index);
      objectRtree.insert(std::make_pair(newObject.bounding_box, next_index));
      next_index++;
  }

  bool updateObject(size_t id, const SemanticObject &newObject)
  {
      SemanticObject &object = objectList[id];
      if (newObject.name != object.name) // update keyword map
      {
          objectMap[object.name].erase(id);
          object.name = newObject.name;
          objectMap[object.name].insert(id);
      }
      if (!bg::equals(newObject.shape, object.shape))
      {
          object.shape = newObject.shape;
          if (!bg::equals(newObject.bounding_box, object.bounding_box)) // update rtree
          {
              objectRtree.remove(std::make_pair(object.bounding_box, id));
              object.bounding_box = newObject.bounding_box;
              objectRtree.insert(std::make_pair(object.bounding_box, id));
          }
      }
      return true;
  }

  bool removeObject(size_t id)
  {
      SemanticObject &object = objectList[id];
      objectMap[object.name].erase(id);
      objectRtree.remove(std::make_pair(object.bounding_box, id));
      objectList.erase(id);
      return true;
  }

  geometry_msgs::Point boostToPointMsg(const point &p)
  {
      geometry_msgs::Point pm;
      pm.x = p.x(); //p.get<0>();
      pm.y = p.y(); //p.get<1>();
      return pm;
  }

  geometry_msgs::Point32 boostToPoint32Msg(const point &p)
  {
      geometry_msgs::Point32 pm;
      pm.x = p.get<0>();
      pm.y = p.get<1>();
      return pm;
  }

  point pointMsgToBoost(const geometry_msgs::Point &pm)
  {
      return point(pm.x, pm.y);
  }

  point point32MsgToBoost(const geometry_msgs::Point32 &pm)
  {
      return point(pm.x, pm.y);
  }

  polygon polygonMsgToBoost(const geometry_msgs::Polygon &pgm)
  {
      polygon pg;
      for (const auto &p : pgm.points)
        bg::append(pg.outer(), point32MsgToBoost(p));
      return pg;
  }

public:
  SemanticLayer() : MapLayerBase("map"), next_index(0) {}

  virtual int getIntValue(double xPos, double yPos);
  virtual std::string getStringValue(double xPos, double yPos);
  virtual void loadMapData();

  std::set<size_t> getObjectsInRange(double xmin, double ymin, double xmax, double ymax)
  {
      return getObjectsInRange(point(xmin, ymin), point(xmax, ymax));
  }

  std::set<size_t> getObjectsInRange(const point &minCorner, const point &maxCorner)
  {
      box query_box(minCorner, maxCorner);
      std::vector<value> result_obj;
      std::set<size_t> result;
      objectRtree.query(bgi::intersects(query_box), std::back_inserter(result_obj));
      for (value val : result_obj)
      {
          const SemanticObject &foundObject = objectList[val.second];
          if (bg::intersects(query_box, foundObject.shape))
              result.insert(val.second);
      }
      return result;
  }

  std::set<size_t> getObjectsInRange(const polygon &pg)
  {
      std::vector<value> result_obj;
      std::set<size_t> result;
      objectRtree.query(bgi::intersects(pg), std::back_inserter(result_obj));
      for (value val : result_obj)
      {
          const SemanticObject &foundObject = objectList[val.second];
          if (bg::intersects(pg, foundObject.shape))
              result.insert(val.second);
      }
      return result;
  }

  std::set<size_t> getObjectsByName(const std::string &name)
  {
      return objectMap[name];
  }

  std::set<size_t> getObjectsByNameInRange(const std::string &name, const point &minCorner, const point &maxCorner)
  {
      auto nameSet = getObjectsByName(name);
      auto rangeSet = getObjectsInRange(minCorner, maxCorner);
      std::set<size_t> intersection;
      std::set_intersection(nameSet.begin(), nameSet.end(), rangeSet.begin(), rangeSet.end(), std::inserter(intersection, intersection.end()));
      return intersection;
  }

  std::vector<std::pair<geometry_msgs::Point, std::string>> getStringReps(const geometry_msgs::Polygon &area)
  {
      std::set<size_t> qres = getObjectsInRange(polygonMsgToBoost(area));
      std::vector<std::pair<geometry_msgs::Point, std::string>> res;
      for (size_t i : qres)
      {
          const SemanticObject &obj = objectList[i];
          point center;
          bg::centroid(obj.shape, center);
          res.push_back(std::make_pair(boostToPointMsg(center), obj.name));
      }
      return res;
  }

  void addExampleObject()
  {
      polygon testPoly;
      bg::append(testPoly.outer(), point(0.0, 0.0));
      bg::append(testPoly.outer(), point(0.0, 1.0));
      bg::append(testPoly.outer(), point(1.0, 0.0));
      box bb = bg::return_envelope<box>(testPoly);
      SemanticObject obj = {"TestTriangle", testPoly, bb};
      /*objectList.push_back(obj);
      objectRtree.insert(std::make_pair(bb, objectList.size() - 1));
      objectMap[obj.name].insert(objectList.size() - 1);*/
      addObject(obj);
  }

  void printQuery()
  {
      box query_box(point(0.5, 0.51), point(5, 5));
      polygon testPoly;
      bg::append(testPoly.outer(), point(0.0, 0.0));
      bg::append(testPoly.outer(), point(0.0, 1.0));
      bg::append(testPoly.outer(), point(1.0, 0.0));
      std::vector<value> result_s;
      objectRtree.query(bgi::intersects(query_box), std::back_inserter(result_s));
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
