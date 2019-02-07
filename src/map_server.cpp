#include "ros/ros.h"
#include "maplayerbase.h"
#include "semanticlayer.h"
#include "hypermap.h"

#include "hypermap/HypermapImage.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Polygon.h"

#include "hypermap/RetrieveStrVals.h"

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

bool retrieveStrVals(hypermap::RetrieveStrVals::Request &req, hypermap::RetrieveStrVals::Response &res)
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

  map->testZip();

  ros::ServiceServer service = nh.advertiseService("retrieve_string_vals", retrieveStrVals);
  //ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
  //ROS_INFO("Ready to add two ints.");
  ros::spin();

  delete map;
  return 0;
}
