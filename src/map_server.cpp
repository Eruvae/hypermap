#include "ros/ros.h"
#include "maplayerbase.h"
#include "semanticlayer.h"

//#include "beginner_tutorials/AddTwoInts.h"

/*bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "map_server");
  ros::NodeHandle nh;

  ROS_INFO("Map server initialized");

  SemanticLayer layer;
  layer.addExampleObject();
  layer.printQuery();

  //ros::ServiceServer service = nh.advertiseService("add_two_ints", add);
  //ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
