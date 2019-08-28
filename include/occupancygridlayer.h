#ifndef OCCUPANCYGRIDLAYER_H
#define OCCUPANCYGRIDLAYER_H

#include <string>
#include <cmath>

#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <ros/service_server.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "nav_msgs/GetMap.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "map_msgs/OccupancyGridUpdate.h"

#include "maplayerbase.h"
#include "boost_geometry_msgs.h"

namespace hypermap
{

class OccupancyGridLayer : public MapLayerBase
{
public:
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

    struct MapFileMetaData
    {
        std::string image = "";
        double resolution;
        double origin[3]; // x, y, yaw
        double occupied_thresh;
        double free_thresh;
        bool negate;
        MapMode mode = TRINARY;
    };

    struct MapIndex
    {
        int x, y;

        bool operator == (const MapIndex &r) const
        {
            return x == r.x && y == r.y;
        }

        bool operator != (const MapIndex &r) const
        {
            return x != r.x || y != r.y;
        }

        MapIndex operator + (const MapIndex &r) const
        {
            return {x + r.x, y + r.y};
        }
        MapIndex operator - (const MapIndex &r) const
        {
            return {x - r.x, y - r.y};
        }
    };
private:

  ros::Publisher mapPub;
  ros::Publisher mapMetaPub;
  ros::ServiceServer mapService;

  ros::Publisher globalMapPub;
  ros::Publisher globalMapMetaPub;
  ros::ServiceServer globalMapService;

  ros::Subscriber mapSub;
  ros::Subscriber mapMetaSub;
  ros::Subscriber mapUpdateSub;

  nav_msgs::GetMap::Response resp;
  nav_msgs::OccupancyGrid &map = resp.map;

  MapFileMetaData fileData;
  /*std::string mapfname = "";
  double res;
  double origin[3]; // x, y, yaw
  int negate;
  double occ_th, free_th;
  MapMode mode = TRINARY;* pixels;*/


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
          publishData();
      }
  }

  void updateMapMeta(const nav_msgs::MapMetaDataConstPtr &new_meta)
  {
      if (subscribe_mode)
      {
          map.info = *new_meta;
          publishData();
      }
  }

  void updateGrid(const map_msgs::OccupancyGridUpdatePtr &update)
  {
      if (enable_update)
      {
          // TODO: increase map size if necessary
          for (int j = 0; j < update->height; j++)
          {
              memcpy(&map.data[getDataIndex(update->x, update->y + j)], &(update->data[j*update->width]), update->width);
          }
      }
  }

  bool mapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);

public:
  OccupancyGridLayer(Hypermap *parent = 0, const std::string &name = "OccupancyGridLayer", const std::string &tfFrame = "map", bool subscribe_mode = false, bool enable_update = true,
                     bool publish_global_topics = false, const std::string &subscribe_topic = "/map");

  virtual ~OccupancyGridLayer()
  {
      ROS_INFO("Occupancy Grid layer destroyed");
  }

  virtual int getIntValue(const geometry_msgs::Point &p);
  virtual std::string getStringValue(const geometry_msgs::Point &p);
  virtual std::vector<std::pair<geometry_msgs::Point, std::string>> getStringValues(const geometry_msgs::Polygon &area);
  virtual std::vector<std::pair<geometry_msgs::Point, int>> getIntValues(const geometry_msgs::Polygon &area);
  virtual std::vector<geometry_msgs::Point> getCoords(const std::string &rep, geometry_msgs::Polygon::ConstPtr area);
  virtual std::vector<geometry_msgs::Point> getCoords(int rep, geometry_msgs::Polygon::ConstPtr area);

  virtual void setSubscribeMode(bool mode);
  virtual void loadMapData(const std::string &file_name);
  virtual void saveMapData();
  virtual void publishData();

  double getYaw()
  {
      tf2::Quaternion quat_tf;
      tf2::convert(map.info.origin.orientation, quat_tf);
      tf2::Matrix3x3 mat(quat_tf);
      double roll, pitch, yaw;
      mat.getRPY(roll, pitch, yaw);
      return yaw;
  }

  MapIndex getPointIndex(const point &p)
  {
      double xDif = p.x() - map.info.origin.position.x;
      double yDif = p.y() - map.info.origin.position.y;
      double angOrig = std::atan2(yDif, xDif);
      double mapAng = angOrig - getYaw();
      double dist = std::sqrt(xDif*xDif + yDif*yDif);
      double mapDifX = dist * cos(mapAng);
      double mapDifY = dist * sin(mapAng);

      return {(int) (mapDifX / map.info.resolution), (int) (mapDifY / map.info.resolution)};
  }

  MapIndex getPointIndex(const geometry_msgs::Point &p)
  {
      double xDif = p.x - map.info.origin.position.x;
      double yDif = p.y - map.info.origin.position.y;
      double angOrig = std::atan2(yDif, xDif);
      double mapAng = angOrig - getYaw();
      double dist = std::sqrt(xDif*xDif + yDif*yDif);
      double mapDifX = dist * cos(mapAng);
      double mapDifY = dist * sin(mapAng);

      return {(int) (mapDifX / map.info.resolution), (int) (mapDifY / map.info.resolution)};
  }

  geometry_msgs::Point getCoordinatesMsg(const MapIndex &index)
  {
      geometry_msgs::Point p;
      double x_loc = index.x * map.info.resolution;
      double y_loc = index.y * map.info.resolution;
      double yaw = getYaw();
      p.x = map.info.origin.position.x + x_loc * cos(yaw) - y_loc * sin(yaw);
      p.y = map.info.origin.position.y + x_loc * sin(yaw) + y_loc * cos(yaw);
      return p;
  }

  point getCoordinates(size_t i, size_t j)
  {
      double x_loc = i * map.info.resolution;
      double y_loc = j * map.info.resolution;
      double yaw = getYaw();
      return point(map.info.origin.position.x + x_loc * cos(yaw) - y_loc * sin(yaw),
                   map.info.origin.position.y + x_loc * sin(yaw) + y_loc * cos(yaw));
  }

  point getCoordinates(const MapIndex &index)
  {
      double x_loc = index.x * map.info.resolution;
      double y_loc = index.y * map.info.resolution;
      double yaw = getYaw();
      return point(map.info.origin.position.x + x_loc * cos(yaw) - y_loc * sin(yaw),
                   map.info.origin.position.y + x_loc * sin(yaw) + y_loc * cos(yaw));
  }

  std::string getGridString(const MapIndex &ind);

  int8_t getGridData(const MapIndex &ind)
  {
      if (!isValid(ind))
      {
          ROS_INFO("Tried to access pixel out of map range.");
          return -1;
      }

      return map.data[getDataIndex(ind)];
  }

  bool setGridData(const MapIndex &ind, int8_t data)
  {
      if (!isValid(ind))
      {
          ROS_INFO("Tried to access pixel out of map range.");
          return false;
      }
      map.data[getDataIndex(ind)] = data;
  }

  std::vector<MapIndex> getGridIndices(const geometry_msgs::Polygon &pgm)
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

  std::vector<MapIndex> getEdgeGridIndices(const geometry_msgs::Polygon &pgm)
  {
      std::vector<MapIndex> res;
      polygon pg = polygonMsgToBoost(pgm);
      for (size_t i = 0; i < pg.outer().size() - 1; i++)
      {
          const MapIndex &ps = getPointIndex(pg.outer()[i]);
          const MapIndex &pe = getPointIndex(pg.outer()[i + 1]);

          ROS_INFO_STREAM("Computing line between {" << ps.x << ", " << ps.y << "} and {" << pe.x << ", " << pe.y << "}");

          int x0 = ps.x, x1 = pe.x;
          int y0 = ps.y, y1 = pe.y;

          // Line draw algorithm; source: http://members.chello.at/~easyfilter/Bresenham.pdf
          int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
          int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
          int err = dx + dy, e2;

          while (true)
          {
              res.push_back({x0, y0});
              e2 = 2*err;
              ROS_INFO_STREAM("Push back cell {" << x0 << ", " << y0 << "}");
              if (e2 >= dy)
              {
                  if (x0 == x1) break;
                  err += dy; x0 += sx;
              }
              if (e2 <= dx)
              {
                  if (y0 == y1) break;
                  err += dx; y0 += sy;
              }
          }
      }
      return res;
  }

  std::vector<MapIndex> getValidGridIndices(const geometry_msgs::Polygon &pgm)
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
              if (isValid(ind) && bg::covered_by(getCoordinates(ind), pg))
              {
                  res.push_back(ind);
              }
          }
      }
      return res;
  }

  std::vector<MapIndex> getValidEdgeGridIndices(const geometry_msgs::Polygon &pgm)
  {
      std::vector<MapIndex> res;
      polygon pg = polygonMsgToBoost(pgm);
      for (size_t i = 0; i < pg.outer().size() - 1; i++)
      {
          const MapIndex &ps = getPointIndex(pg.outer()[i]);
          const MapIndex &pe = getPointIndex(pg.outer()[i + 1]);

          ROS_INFO_STREAM("Computing line between {" << ps.x << ", " << ps.y << "} and {" << pe.x << ", " << pe.y << "}");

          int x0 = ps.x, x1 = pe.x;
          int y0 = ps.y, y1 = pe.y;

          // Line draw algorithm; source: http://members.chello.at/~easyfilter/Bresenham.pdf
          int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
          int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
          int err = dx + dy, e2;

          while (true)
          {
              MapIndex ind {x0, y0};
              if (isValid(ind))
                  res.push_back(ind);

              e2 = 2*err;
              ROS_INFO_STREAM("Push back cell {" << x0 << ", " << y0 << "}");
              if (e2 >= dy)
              {
                  if (x0 == x1) break;
                  err += dy; x0 += sx;
              }
              if (e2 <= dx)
              {
                  if (y0 == y1) break;
                  err += dx; y0 += sy;
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

  const nav_msgs::MapMetaData &getMapMeta()
  {
      return map.info;
  }

  const nav_msgs::OccupancyGrid &getMap()
  {
      return map;
  }

  void setMapMeta(const nav_msgs::MapMetaData &mapMeta)
  {
      map.info = mapMeta;
  }

  void setMap(const nav_msgs::OccupancyGrid &map)
  {
      this->map = map;
  }

  void createEmptyMap(const nav_msgs::MapMetaData &info)
  {
      map.info = info;
      map.data.assign(info.height*info.width, -1);
      publishData();
  }

  bool loadMapMeta(std::istream &in);
  bool loadMap(const std::string &data);

  bool saveMapMeta(std::ostream &out);
  bool saveMap(std::ostream &out);
};

}

#endif // OCCUPANCYGRIDLAYER_H
