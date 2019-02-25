#ifndef OCCUPANCYGRIDLAYER_H
#define OCCUPANCYGRIDLAYER_H

#include <string>

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/service_server.h>

#include "nav_msgs/GetMap.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"

#include "maplayerbase.h"
#include "convert_boost.h"

namespace hypermap
{

class OccupancyGridLayer : public MapLayerBase
{
    /** Map mode
     *  Default: TRINARY -
     *      value >= occ_th - Occupied (100)
     *      value <= free_th - Free (0)
     *      otherwise - Unknown
     *  SCALE -
     *      alpha < 1.0 - Unknown
     *      value >= occ_th - Occupied (100)
     *      value <= free_th - Free (0)
     *      otherwise - f( (free_th, occ_th) ) = (0, 100)
     *          (linearly map in between values to (0,100)
     *  RAW -
     *      value = value
     */
    enum MapMode {TRINARY, SCALE, RAW};

    struct MapIndex
    {
        int x, y;

        bool operator == (const MapIndex &r)
        {
            return x == r.x && y == r.y;
        }

        bool operator != (const MapIndex &r)
        {
            return x != r.x || y != r.y;
        }

        MapIndex operator + (const MapIndex &r)
        {
            return {x + r.x, y + r.y};
        }
        MapIndex operator - (const MapIndex &r)
        {
            return {x - r.x, y - r.y};
        }
    };

  ros::Publisher mapPub;
  ros::Publisher mapMetaPub;
  ros::ServiceServer mapService;

  ros::Subscriber mapSub;
  ros::Subscriber mapMetaSub;

  nav_msgs::GetMap::Response resp;
  nav_msgs::OccupancyGrid &map = resp.map;

  std::string name = "";
  std::string mapfname = "";
  double res;
  double origin[3]; // x, y, yaw
  int negate;
  double occ_th, free_th;
  MapMode mode = TRINARY;

  inline size_t getDataIndex(size_t i, size_t j) // compute linear index for given map coords
  {
      return map.info.width * j + i;
  }

  inline size_t getDataIndex(const MapIndex &ind) // compute linear index for given map coords
  {
      return map.info.width * ind.y + ind.x;
  }

  inline bool isValid(const MapIndex &ind)
  {
      return (ind.x >= 0 && ind.x < map.info.width && ind.y >= 0 && ind.y < map.info.height);
  }

  void updateMap(const nav_msgs::OccupancyGridConstPtr &new_grid)
  {
      if (subscribe_mode)
      {
          map = *new_grid;
      }
  }

  void updateMapMeta(const nav_msgs::MapMetaDataConstPtr &new_meta)
  {
      if (subscribe_mode)
      {
          map.info = *new_meta;
      }
  }

  bool mapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);

public:
  OccupancyGridLayer(Hypermap *parent = 0);

  virtual int getIntValue(const geometry_msgs::Point &p);
  virtual std::string getStringValue(const geometry_msgs::Point &p);
  virtual void setSubscribeMode(bool mode);
  virtual void loadMapData(const std::string &file_name);
  virtual void saveMapData();
  virtual void publishData();

  std::vector<std::tuple<geometry_msgs::Point, std::string>> getStringRep(geometry_msgs::Polygon::ConstPtr area)
  {
      std::vector<MapIndex> inds = getGridIndices(*area);
      std::vector<std::tuple<geometry_msgs::Point, std::string>> res;
      for (const MapIndex &ind : inds)
      {
          res.push_back(std::make_tuple(getCoordinatesMsg(ind), getGridString(ind)));
      }
      return res;
  }

  std::vector<geometry_msgs::Point> getCoords(const std::string &rep, geometry_msgs::Polygon::ConstPtr area)
  {
      std::vector<geometry_msgs::Point> res;
      if(area)
      {
          std::vector<MapIndex> inds = getGridIndices(*area);
          for (const MapIndex &ind : inds)
          {
              if (rep == getGridString(ind))
              {
                  res.push_back(getCoordinatesMsg(ind));
              }
          }
      }
      else
      {
          for (int i = 0; i < map.info.width; i++)
          {
              for (int j = 0; j < map.info.height; j++)
              {
                  MapIndex ind = {i, j};
                  if (rep == getGridString(ind))
                  {
                      res.push_back(getCoordinatesMsg(ind));
                  }
              }
          }
      }
      return res;
  }

  MapIndex getPointIndex(const point &p)
  {
      double xDif = p.x() - origin[0];
      double yDif = p.y() - origin[1];
      double angOrig = atan2(yDif, xDif);
      double mapAng = angOrig - origin[2];
      double dist = sqrt(xDif*xDif + yDif*yDif);
      double mapDifX = dist * cos(mapAng);
      double mapDifY = dist * sin(mapAng);

      return {(int) (mapDifX / res), (int) (mapDifY / res)};
  }

  MapIndex getPointIndex(const geometry_msgs::Point &p)
  {
      double xDif = p.x - origin[0];
      double yDif = p.y - origin[1];
      double angOrig = atan2(yDif, xDif);
      double mapAng = angOrig - origin[2];
      double dist = sqrt(xDif*xDif + yDif*yDif);
      double mapDifX = dist * cos(mapAng);
      double mapDifY = dist * sin(mapAng);

      return {(int) (mapDifX / res), (int) (mapDifY / res)};
  }

  geometry_msgs::Point getCoordinatesMsg(const MapIndex &index)
  {
      geometry_msgs::Point p;
      double x_loc = index.x * res;
      double y_loc = index.y * res;
      p.x = origin[0] + x_loc * cos(origin[2]) - y_loc * sin(origin[2]);
      p.y = origin[1] + x_loc * sin(origin[2]) + y_loc * cos(origin[2]);
      return p;
  }

  point getCoordinates(size_t i, size_t j)
  {
      double x_loc = i * res;
      double y_loc = j * res;
      return point(origin[0] + x_loc * cos(origin[2]) - y_loc * sin(origin[2]),
                   origin[1] + x_loc * sin(origin[2]) + y_loc * cos(origin[2]));
  }

  point getCoordinates(const MapIndex &index)
  {
      double x_loc = index.x * res;
      double y_loc = index.y * res;
      return point(origin[0] + x_loc * cos(origin[2]) - y_loc * sin(origin[2]),
                   origin[1] + x_loc * sin(origin[2]) + y_loc * cos(origin[2]));
  }

  std::string getGridString(const MapIndex &ind);

  int8_t getGridData(const MapIndex &ind)
  {
      if (!isValid(ind))
          return -1;

      return getDataIndex(ind);
  }

  std::vector<MapIndex> getGridIndices(const geometry_msgs::Polygon pgm)
  {
      std::vector<MapIndex> res;
      polygon pg = polygonMsgToBoost(pgm);
      box bb = bg::return_envelope<box>(pg);
      MapIndex minCorner = getPointIndex(bb.min_corner());
      MapIndex maxCorner = getPointIndex(bb.max_corner());
      for (int i = minCorner.x; i < maxCorner.x; i++)
      {
          for (int j = minCorner.y; j < maxCorner.y; j++)
          {
              MapIndex ind {i, j};
              if (bg::covered_by(getCoordinates(ind), pg))
              {
                  res.push_back(ind);
              }
          }
      }
      return res;
  }

  /*std::vector<MapIndex> getGridIndices(const geometry_msgs::Point &bottomLeft, const geometry_msgs::Point &topLeft, const geometry_msgs::Point &bottomRight)
  {
      MapIndex bL = getPointIndices(bottomLeft);
      MapIndex tL = getPointIndices(topLeft);
      MapIndex bR = getPointIndices(bottomRight);
      MapIndex tR = tL + bR - bL;

      MapIndex current = bL;
      while (current != tR)
      {

      }

  }*/

  bool loadMapMeta(std::istream &in);
  bool loadMap(const std::string &data);

  bool saveMapMeta(std::ostream &out);
  bool saveMap(std::ostream &out);
};

}

#endif // OCCUPANCYGRIDLAYER_H
