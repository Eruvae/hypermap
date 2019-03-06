#ifndef HYPERMAP_H
#define HYPERMAP_H

#include <string>
#include <vector>
#include <map>
#include <functional>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "zip.hpp"
//#include "libzippp.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PolygonStamped.h"

#include "maplayerbase.h"

namespace hypermap
{

//using namespace libzippp;

class Hypermap
{
  //template<typename T> static std::unique_ptr<MapLayerBase> createLayer() { return std::unique_ptr<MapLayerBase>(new T); }
  //static const std::map<std::string, std::unique_ptr<MapLayerBase> (*) ()> layerClassMap = {{"OccupancyGrid", <OccupancyGridLayer>createLayer()}, {"Semantic", <SemanticLayer>createLayer()}};

  std::map<std::string, size_t> strToInd;
  std::vector<std::unique_ptr<MapLayerBase>> layers;
  //ZipArchive *mapFile;
  //libzip::archive *mapFile;
  std::unique_ptr<libzip::archive> mapFile;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

public:
  ros::NodeHandle &nh;
  Hypermap(ros::NodeHandle &nh) : nh(nh), tfListener(tfBuffer)/*, mapFile(0)*/ {}

  void clear();

  size_t getLayerCnt()
  {
      return layers.size();
  }

  MapLayerBase* getLayer(size_t ind)
  {
      if (ind >= layers.size())
      {
          ROS_ERROR("Layer index out of range");
          return 0;
      }
      return layers[ind].get();
  }

  MapLayerBase* getLayer(const std::string &name)
  {
      auto it = strToInd.find(name);
      if (it == strToInd.end())
      {
          ROS_ERROR_STREAM("Layer " << name << " not found");
          return 0;
      }
      size_t ind = it->second;
      return layers[ind].get();
  }

  std::string getStringValue(const std::string &layer, const geometry_msgs::Point &p)
  {
      MapLayerBase *lp = getLayer(layer);
      if (lp == 0)
          return "";

      return lp->getStringValue(p);
  }

  std::vector<std::pair<geometry_msgs::Point, std::string>> getStringValues(const std::string &layer, const geometry_msgs::Polygon &area)
  {
      MapLayerBase *lp = getLayer(layer);
      if (lp == 0)
          return std::vector<std::pair<geometry_msgs::Point, std::string>>();

      return lp->getStringValues(area);
  }

  std::vector<geometry_msgs::Point> getCoords(const std::string &layer, const std::string &rep, geometry_msgs::Polygon::ConstPtr area = 0)
  {
      MapLayerBase *lp = getLayer(layer);
      if (lp == 0)
          return std::vector<geometry_msgs::Point>();

      return lp->getCoords(rep, area);
  }

  void publishLayerData();

  bool loadMapFile(const std::string &path);

  bool saveMapFile(const std::string &path);

  inline void closeMapFile()
  {
      /*mapFile->close();
      delete mapFile;
      mapFile = 0;*/

      /*delete mapFile;
      mapFile = 0;*/
      mapFile.reset();
  }

  void loadMapConfig(std::istream &data);

  void saveMapConfig(std::ostream &out);

  void transformPoint(geometry_msgs::PointStamped &p, const std::string &target);

  //void transformPolygon(geometry_msgs::PolygonStamped &p, const std::string &target);

  std::string getLayerFile(const std::string &fname);
  bool getLayerFile(const std::string &fname, std::function<bool(std::istream&)> getter);
  bool getLayerFile(const std::string &fname, std::function<bool(const std::string&)> getter);

  bool putLayerFile(const std::string &fname, const std::string &data);
  bool putLayerFile(const std::string &fname, std::function<bool(std::ostream&)> putter);
  bool putLayerFile(const std::string &fname, std::function<std::string()> putter);

  void testZip();
};

}

#endif // HYPERMAP_H
