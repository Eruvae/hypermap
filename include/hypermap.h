#ifndef HYPERMAP_H
#define HYPERMAP_H

#include <string>
#include <vector>
#include <map>
#include <functional>

#include <ros/ros.h>
#include "zip.hpp"
//#include "libzippp.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Polygon.h"

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

public:
  ros::NodeHandle &nh;
  Hypermap(ros::NodeHandle &nh) : nh(nh)/*, mapFile(0)*/ {}

  size_t getLayerCnt()
  {
      return layers.size();
  }

  MapLayerBase* getLayer(size_t ind)
  {
      return layers[ind].get();
  }

  bool loadMapFile(const std::string &path);

  bool saveMapFile(const std::string &path);

  void closeMapFile();

  void loadMapConfig(std::istream &data);

  void saveMapConfig(std::ostream &out);

  void transformPoint(geometry_msgs::Point &p, const std::string &origin, const std::string &target);

  void transformPolygon(geometry_msgs::Polygon &p, const std::string &origin, const std::string &target);

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
