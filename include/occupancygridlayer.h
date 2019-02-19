#ifndef OCCUPANCYGRIDLAYER_H
#define OCCUPANCYGRIDLAYER_H

#include "ros/ros.h"
#include "maplayerbase.h"
#include "nav_msgs/OccupancyGrid.h"

class OccupancyGridLayer : public MapLayerBase
{
  ros::Publisher mapPub;
  ros::Publisher mapMetaPub;

  ros::Subscriber mapSub;
  ros::Subscriber mapMetaSub;

  nav_msgs::MapMetaData metaData;
  nav_msgs::OccupancyGrid grid;

  void updateMap(const nav_msgs::OccupancyGridConstPtr &new_grid)
  {
      if (subscribe_mode)
      {
          grid = *new_grid;
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
  virtual void publishData();
};

#endif // OCCUPANCYGRIDLAYER_H
