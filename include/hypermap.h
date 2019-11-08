#ifndef HYPERMAP_H
#define HYPERMAP_H

#include <string>
#include <vector>
#include <map>
#include <functional>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
//#include "zip.hpp"
#include "zip_mz.hpp"
//#include "libzippp.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PolygonStamped.h"
#include "geometry_msgs/TransformStamped.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_polygon_msgs.h"

#include "hypermap_msgs/HypermapMetaData.h"
#include "hypermap_msgs/LayerMetaData.h"

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
  //std::unique_ptr<libzip::archive> mapFile;
  std::unique_ptr<miniz::zip_reader> mapReader;
  std::unique_ptr<miniz::zip_writer> mapWriter;

  hypermap_msgs::HypermapMetaData metaData;
  ros::Publisher metaDataPub;

  std::vector<geometry_msgs::TransformStamped> transforms;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  tf2_ros::TransformBroadcaster tfBroadcaster;

public:
  ros::NodeHandle &nh;
  Hypermap(ros::NodeHandle &nh) : nh(nh), tfListener(tfBuffer)/*, mapFile(0)*/
  {
      metaDataPub = nh.advertise<hypermap_msgs::HypermapMetaData>("hypermap_metadata", 1, true);
  }

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

  void addLayer(const std::string type, const std::string name, const std::string frame_id, bool subscribe_mode = false, bool enable_update = true, bool publish_global_topics = false);

  int getIntValue(const std::string &layer, geometry_msgs::PointStamped &p);
  std::string getStringValue(const std::string &layer, geometry_msgs::PointStamped &p);
  std::vector<std::pair<geometry_msgs::Point, int>> getIntValues(const std::string &layer, geometry_msgs::PolygonStamped &area);
  std::vector<std::pair<geometry_msgs::Point, std::string>> getStringValues(const std::string &layer, geometry_msgs::PolygonStamped &area);
  std::vector<geometry_msgs::Point> getCoords(const std::string &layer, int rep, geometry_msgs::PolygonStamped::Ptr area = 0);
  std::vector<geometry_msgs::Point> getCoords(const std::string &layer, const std::string &rep, geometry_msgs::PolygonStamped::Ptr area = 0);

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
      mapReader.reset();
      mapWriter.reset();
  }

  void loadMapConfig(std::istream &data);

  void saveMapConfig(std::ostream &out);

  bool transform(geometry_msgs::PointStamped &p, const std::string &target);
  bool transform(geometry_msgs::PolygonStamped &p, const std::string &target);
  bool transform(geometry_msgs::Point &p, const std::string &target, const std::string &fixed);
  bool transform(geometry_msgs::Polygon &p, const std::string &target, const std::string &fixed);

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
