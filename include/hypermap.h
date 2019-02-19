#ifndef HYPERMAP_H
#define HYPERMAP_H

#include "ros/ros.h"
#include "maplayerbase.h"
#include "occupancygridlayer.h"
#include "semanticlayer.h"
//#include "libzippp.h"
#include "zip.hpp"
#include "yaml-cpp/yaml.h"
#include <boost/assign.hpp>
#include <string>
#include <vector>
#include <map>

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

  void loadMapFile(const char *path);

  void closeMapFile();

public:
  ros::NodeHandle &nh;
  Hypermap(ros::NodeHandle &nh) : nh(nh)/*, mapFile(0)*/ {}

  size_t getLayerCnt()
  {
      return layers.size();
  }

  void loadMapConfig(const std::string &data);

  void transformPoint(geometry_msgs::Point &p, const std::string &origin, const std::string &target);

  void transformPolygon(geometry_msgs::Polygon &p, const std::string &origin, const std::string &target);

  std::string getLayerFile(const char *fname);

  void testZip();
};

#endif // HYPERMAP_H
