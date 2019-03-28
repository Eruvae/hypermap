#include <fstream>
#include <csignal>

#include <ros/ros.h>

#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PolygonStamped.h"

#include "hypermap_msgs/GetIntAtPoint.h"
#include "hypermap_msgs/GetIntsByArea.h"
#include "hypermap_msgs/GetLocationsByInt.h"
#include "hypermap_msgs/GetLocationsByString.h"
#include "hypermap_msgs/GetStringAtPoint.h"
#include "hypermap_msgs/GetStringsByArea.h"

#include "semanticlayer.h"
#include "hypermap.h"

hypermap::Hypermap *map;

bool getIntAtPoint(hypermap_msgs::GetIntAtPoint::Request &req, hypermap_msgs::GetIntAtPoint::Response &res)
{
    res.value = map->getIntValue(req.layer, req.location);
    return true;
}

bool getIntsByArea(hypermap_msgs::GetIntsByArea::Request &req, hypermap_msgs::GetIntsByArea::Response &res)
{
    auto vals = map->getIntValues(req.layer, req.area);
    for (const auto &val : vals)
    {
        res.locations.push_back(val.first);
        res.values.push_back(val.second);
    }
    return true;
}

bool getLocationsByInt(hypermap_msgs::GetLocationsByInt::Request &req, hypermap_msgs::GetLocationsByInt::Response &res)
{
    if (req.area.polygon.points.empty())
        res.locations = map->getCoords(req.layer, req.value);
    else
    {
        geometry_msgs::PolygonStamped::Ptr area(new geometry_msgs::PolygonStamped);
        *area = req.area;
        res.locations = map->getCoords(req.layer, req.value, area);
    }
    return true;
}

bool getLocationsByString(hypermap_msgs::GetLocationsByString::Request &req, hypermap_msgs::GetLocationsByString::Response &res)
{
    if (req.area.polygon.points.empty())
        res.locations = map->getCoords(req.layer, req.name);
    else
    {
        geometry_msgs::PolygonStamped::Ptr area(new geometry_msgs::PolygonStamped);
        *area = req.area;
        res.locations = map->getCoords(req.layer, req.name, area);
    }
    return true;
}

bool getStringAtPoint(hypermap_msgs::GetStringAtPoint::Request &req, hypermap_msgs::GetStringAtPoint::Response &res)
{
    res.name = map->getStringValue(req.layer, req.location);
    return true;
}

bool getStringsByArea(hypermap_msgs::GetStringsByArea::Request &req, hypermap_msgs::GetStringsByArea::Response &res)
{
    auto vals = map->getStringValues(req.layer, req.area);
    for (const auto &val : vals)
    {
        res.locations.push_back(val.first);
        res.names.push_back(val.second);
    }
    return true;
}

std::string readNext(const std::string &str, std::string::const_iterator &begin)
{
    std::string::const_iterator end = str.cend();
    std::string res;
    res.reserve(str.size());

    for (; begin != end && *begin == ' '; begin++); // skip leading whitespaces

    char end_char = ' ';
    if (begin != end && *begin == '"')
    {
        end_char = '"';
        begin++;
    }

    bool escape = false;
    for(; begin != end; begin++)
    {
        if (escape)
            escape = false;
        else if (*begin == '\\')
        {
            escape = true;
            continue;
        }
        else if (*begin == end_char)
        {
            begin++; // remove end character
            break;
        }
        res += *begin;
    }
    return res;
}

void sigintHandler(int sig)
{
    delete map;
    exit(0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_server", ros::init_options::NoSigintHandler);

  if (argc > 2)
  {
      ROS_ERROR("Too many arguments!");
      return -1;
  }

  ros::NodeHandle nh;
  map = new hypermap::Hypermap(nh);

  if (argc == 2)
  {
      map->loadMapFile(argv[1]);
  }

  signal(SIGINT, sigintHandler);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::ServiceServer getIntAtPointService = nh.advertiseService("get_int_at_point", getIntAtPoint);
  ros::ServiceServer getIntsByAreaService = nh.advertiseService("get_ints_by_area", getIntsByArea);
  ros::ServiceServer getLocationsByIntService = nh.advertiseService("get_locations_by_int", getLocationsByInt);
  ros::ServiceServer getLocationsByStringService = nh.advertiseService("get_locations_by_string", getLocationsByString);
  ros::ServiceServer getStringAtPointService = nh.advertiseService("get_string_at_point", getStringAtPoint);
  ros::ServiceServer getStringsByAreaService = nh.advertiseService("get_strings_by_area", getStringsByArea);

  ROS_INFO("Map server initialized");

  std::cout << "Waiting for input. type \"help\" for help." << std::endl;
  while(1)
  {
      std::string in;
      std::getline(std::cin, in);
      std::string::const_iterator it = in.cbegin();
      std::string command = readNext(in, it);
      if (command == "help")
      {
          std::cout << "Available commands:" << std::endl; // TODO
      }
      else if (command == "load")
      {
          std::string fname = readNext(in, it);
          std::cout << "Loading map file: " << fname << std::endl;
          if (map->loadMapFile(fname))
          {
              std::cout << "Map loaded successfully" << std::endl;
          }
          else
          {
              std::cout << "Failed loading map" << std::endl;
          }
      }
      else if (command == "loadConfig")
      {
          std::string fname = readNext(in, it);
          std::ifstream file(fname);
          if (!file)
          {
              std::cout << "Config file could not be opened" << std::endl;
              continue;
          }
          std::cout << "Loading map config: " << fname << std::endl;
          try
          {
              map->loadMapConfig(file);
              std::cout << "Map loaded successfully" << std::endl;
          }
          catch (const std::runtime_error &e)
          {
              std::cout << "Failed loading map: " << e.what() << std::endl;
          }
          file.close();
      }
      else if (command == "save")
      {
          std::string fname = readNext(in, it);
          std::cout << "Saving map file: " << fname << std::endl;
          if (map->saveMapFile(fname))
          {
              std::cout << "Map saving successfully" << std::endl;
          }
          else
          {
              std::cout << "Failed saving map" << std::endl;
          }
      }
      else if (command == "getString")
      {
          std::string layer = readNext(in, it);
          geometry_msgs::PointStamped p;
          p.header.frame_id = "map";
          p.header.stamp = ros::Time::now();
          p.point.x = std::stod(readNext(in, it));
          p.point.y = std::stod(readNext(in, it));
          std::string value = map->getStringValue(layer, p);
          std::cout << "Value on Layer " << layer << ", Position [" << p.point.x << "; " << p.point.y << "]: " << value << std::endl;
      }
      else if (command == "getStrings")
      {
          std::string layer = readNext(in, it);
          geometry_msgs::PolygonStamped pg;
          pg.header.frame_id = "map";
          pg.header.stamp = ros::Time::now();
          while (it != in.end())
          {
              geometry_msgs::Point32 p;
              p.x = std::stod(readNext(in, it));
              p.y = std::stod(readNext(in, it));
              pg.polygon.points.push_back(p);
          }
          auto values = map->getStringValues(layer, pg);
          std::cout << "Values on Layer " << layer << ", Area: [ ";
          for (const auto &p : pg.polygon.points)
              std::cout << "[" << p.x << "; " << p.y << "] ";

          std::cout << "]:" << std::endl;
          for (const auto &v : values)
          {
              std::cout << "[" << v.first.x << "; " << v.first.y << "]: " << v.second << std::endl;
          }
      }
      else if (command == "getCoords")
      {
          std::string layer = readNext(in, it);
          std::string rep = readNext(in, it);
          std::vector<geometry_msgs::Point> locs = map->getCoords(layer, rep);
          std::cout << "Coordinates of " << rep << ":" << std::endl;
          for (const geometry_msgs::Point &p : locs)
          {
              std::cout << "[" << p.x << "; " << p.y << "]" << std::endl;
          }
      }
      else if (command == "getCoordsArea")
      {
          std::string layer = readNext(in, it);
          std::string rep = readNext(in, it);
          geometry_msgs::PolygonStamped::Ptr pg(new geometry_msgs::PolygonStamped);
          pg->header.frame_id = "map";
          pg->header.stamp = ros::Time::now();
          while (it != in.end())
          {
              geometry_msgs::Point32 p;
              p.x = std::stod(readNext(in, it));
              p.y = std::stod(readNext(in, it));
              pg->polygon.points.push_back(p);
          }
          std::vector<geometry_msgs::Point> locs = map->getCoords(layer, rep, pg);
          std::cout << "Coordinates of " << rep << " in area [ ";
          for (const auto &p : pg->polygon.points)
              std::cout << "[" << p.x << "; " << p.y << "] ";

          std::cout << "]:" << std::endl;
          for (const geometry_msgs::Point &p : locs)
          {
              std::cout << "[" << p.x << "; " << p.y << "]" << std::endl;
          }
      }
      else if (command == "republish")
      {
          map->publishLayerData();
      }
      else if (command == "exit")
      {
          std::cout << "Shutting down" << std::endl;
          break;
      }
      else if (command == "clear")
      {
          map->clear();
      }
      else
      {
          std::cout << "Command \"" << command << "\" not recognized" << std::endl;
      }
  }

  delete map;
  return 0;

  //map->loadMapFile("hmap_example.hmap");

  //hypermap::SemanticLayer *l = (hypermap::SemanticLayer*) map->getLayer(1);
  //l->writeMapData(std::cout);

  //map->saveMapFile("hmap_copy.hmap");
  //map->saveMapConfig(std::cout);

  /*geometry_msgs::Point p;

  layer.addExampleObject();
  layer.printQuery();

  std::ifstream test_semmap("test_semantic.yaml");
  //std::stringstream semmap_stream;
  //semmap_stream << test_semmap.rdbuf();
  //layer.readMapData(semmap_stream.str());

  layer.readMapData(test_semmap);
  test_semmap.close();

  //std::string out = layer.generateMapData();
  std::ofstream out_map("test_semantic_out.yaml");
  //out_map << out;
  layer.writeMapData(out_map);
  out_map.close();

  map->testZip();

  std::cout << "Hmap layer cnt: " << map->getLayerCnt() << std::endl;

  std::ifstream hmap_conf("testhmap_meta.yaml");
  std::stringstream hmap_stream;
  hmap_stream << hmap_conf.rdbuf();
  hmap_conf.close();
  map->loadMapConfig(hmap_stream.str());

  std::cout << "Hmap layer cnt: " << map->getLayerCnt() << std::endl;

  std::string tstr("Hello\0 World", 12);
  const uint8_t *tdata = (const uint8_t*)tstr.data();
  std::cout << tstr.length() << ", " << tdata[3] << ", " << tdata[5] << ", " << tdata[7] << std::endl;
  */
}
