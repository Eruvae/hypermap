#ifndef OCCUPANCYGRIDLAYER_H
#define OCCUPANCYGRIDLAYER_H

#include <string>

#include <ros/ros.h>

#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"

#include "maplayerbase.h"

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

  ros::Publisher mapPub;
  ros::Publisher mapMetaPub;

  ros::Subscriber mapSub;
  ros::Subscriber mapMetaSub;

  nav_msgs::MapMetaData metaData;
  nav_msgs::OccupancyGrid map;

  std::string mapfname = "";
  double res;
  double origin[3];
  int negate;
  double occ_th, free_th;
  MapMode mode = TRINARY;

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
          metaData = *new_meta;
      }
  }

public:
  OccupancyGridLayer(Hypermap *parent = 0);

  virtual int getIntValue(double xPos, double yPos);
  virtual std::string getStringValue(double xPos, double yPos);
  virtual void setSubscribeMode(bool mode);
  virtual void loadMapData(const std::string &file_name);
  virtual void saveMapData();
  virtual void publishData();

  bool loadMapMeta(std::istream &in);
  void loadMap(const std::string &data);

  void saveMapMeta(std::ostream &out);
  void saveMap(std::ostream &out);
};

}

#endif // OCCUPANCYGRIDLAYER_H
