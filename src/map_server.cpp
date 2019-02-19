#include <fstream>

#include "ros/ros.h"
#include "maplayerbase.h"
#include "semanticlayer.h"
#include "hypermap.h"

#include "hypermap_msgs/HypermapImage.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Polygon.h"

#include "hypermap_msgs/RetrieveStrVals.h"

SemanticLayer layer;
Hypermap *map;

//#include "beginner_tutorials/AddTwoInts.h"

/*bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}*/

bool retrieveStrVals(hypermap_msgs::RetrieveStrVals::Request &req, hypermap_msgs::RetrieveStrVals::Response &res)
{
    const auto &qres = layer.getStringReps(req.area);
    for (const auto &obj : qres)
    {
        res.locations.push_back(obj.first);
        res.names.push_back(obj.second);
    }
    return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_server");
  ros::NodeHandle nh;
  map = new Hypermap(nh);

  ROS_INFO("Map server initialized");

  geometry_msgs::Point p;

  layer.addExampleObject();
  layer.printQuery();

  std::ifstream test_semmap("test_semantic.yaml");
  std::stringstream semmap_stream;
  semmap_stream << test_semmap.rdbuf();
  test_semmap.close();

  layer.readMapData(semmap_stream.str());

  std::string out = layer.generateMapData();
  std::ofstream out_map("test_semantic_out.yaml");
  out_map << out;
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

  ros::ServiceServer service = nh.advertiseService("retrieve_string_vals", retrieveStrVals);
  //ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
  //ROS_INFO("Ready to add two ints.");
  ros::spin();

  delete map;
  return 0;
}
