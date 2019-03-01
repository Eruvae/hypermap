#ifndef CONVERT_BOOST_H
#define CONVERT_BOOST_H

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Polygon.h"

namespace hypermap
{

namespace bg = boost::geometry;
namespace bgi = boost::geometry::index;

//typedef bg::model::point<double, 2, bg::cs::cartesian> point;
typedef bg::model::d2::point_xy<double> point;
typedef bg::model::box<point> box;
typedef bg::model::polygon<point> polygon;
typedef bg::model::ring<point> ring;

inline geometry_msgs::Point boostToPointMsg(const point &p)
{
    geometry_msgs::Point pm;
    pm.x = p.x(); //p.get<0>();
    pm.y = p.y(); //p.get<1>();
    return pm;
}

inline geometry_msgs::Point32 boostToPoint32Msg(const point &p)
{
    geometry_msgs::Point32 pm;
    pm.x = p.x(); //get<0>();
    pm.y = p.y(); //get<1>();
    return pm;
}

inline geometry_msgs::Polygon boostToPolygonMsg(const polygon &pg)
{
    geometry_msgs::Polygon pgm;
    for (const auto &p : pg.outer())
        pgm.points.push_back(boostToPoint32Msg(p));

    return pgm;
}

inline point pointMsgToBoost(const geometry_msgs::Point &pm)
{
    return point(pm.x, pm.y);
}

inline point point32MsgToBoost(const geometry_msgs::Point32 &pm)
{
    return point(pm.x, pm.y);
}

inline polygon polygonMsgToBoost(const geometry_msgs::Polygon &pgm)
{
    polygon pg;
    for (const auto &p : pgm.points)
      bg::append(pg.outer(), point32MsgToBoost(p));

    bg::correct(pg);
    return pg;
}

}

#endif // CONVERT_BOOST_H
