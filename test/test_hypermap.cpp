#include <string>
//#include <fstream>
#include <gtest/gtest.h>
#include <ros/package.h>

#include "hypermap.h"
#include "hypermap_msgs/GetIntAtPoint.h"
#include "hypermap_msgs/GetIntsByArea.h"
#include "hypermap_msgs/GetLocationsByInt.h"
#include "hypermap_msgs/GetLocationsByString.h"
#include "hypermap_msgs/GetStringAtPoint.h"
#include "hypermap_msgs/GetStringsByArea.h"

using hypermap::Hypermap;

std::string package_path;
ros::NodeHandle *nh;

TEST(HypermapTests, loadMapFile)
{
    Hypermap hm(*nh);
    hm.loadMapFile(package_path + "/test/hmap_example.hmap");
    ASSERT_EQ(2, hm.getLayerCnt());
}

inline geometry_msgs::Point32 createPoint32(float x, float y)
{
    geometry_msgs::Point32 pm;
    pm.x = x;
    pm.y = y;
    return pm;
}


TEST(HypermapTests, testServices)
{
    //ros::ServiceClient getIntAtPointClient = nh->serviceClient<hypermap_msgs::GetIntAtPoint>("/hypermap_server/get_int_at_point");
    //ros::ServiceClient getIntsByAreaClient = nh->serviceClient<hypermap_msgs::GetIntsByArea>("/hypermap_server/get_ints_by_area");
    //ros::ServiceClient getLocationsByIntClient = nh->serviceClient<hypermap_msgs::GetLocationsByInt>("/hypermap_server/get_locations_by_int");
    ros::ServiceClient getLocationsByStringClient = nh->serviceClient<hypermap_msgs::GetLocationsByString>("/hypermap_server/get_locations_by_string");
    ros::ServiceClient getStringAtPointClient = nh->serviceClient<hypermap_msgs::GetStringAtPoint>("/hypermap_server/get_string_at_point");
    ros::ServiceClient getStringsByAreaClient = nh->serviceClient<hypermap_msgs::GetStringsByArea>("/hypermap_server/get_strings_by_area");

    hypermap_msgs::GetLocationsByString lbs;
    lbs.request.layer = "semantic";
    lbs.request.name = "sink";
    ASSERT_EQ(getLocationsByStringClient.call(lbs), true) << "Failed to call service";
    ASSERT_EQ(lbs.response.locations.size(), 1) << "Result should return exactly one location for sink";
    const geometry_msgs::Point &p1 = lbs.response.locations[0];
    ASSERT_NEAR(p1.x, 3.259, 0.001) << "X coordinate of sink not as expected";
    ASSERT_NEAR(p1.y, 2.378, 0.001) << "Y coordinate of sink not as expected";

    lbs.request.area.header.frame_id = "map";
    lbs.request.area.polygon.points.push_back(createPoint32(2, 2));
    lbs.request.area.polygon.points.push_back(createPoint32(2, 3));
    lbs.request.area.polygon.points.push_back(createPoint32(1, 3));
    lbs.request.area.polygon.points.push_back(createPoint32(1, 2));
    lbs.request.name = "chair";
    ASSERT_EQ(getLocationsByStringClient.call(lbs), true) << "Failed to call service";
    ASSERT_EQ(lbs.response.locations.size(), 1) << "Result should return exactly one location";
    const geometry_msgs::Point &p2 = lbs.response.locations[0];
    ASSERT_NEAR(p2.x, 1.429, 0.001) << "X coordinate of sink not as expected";
    ASSERT_NEAR(p2.y, 2.887, 0.001) << "Y coordinate of sink not as expected";

    hypermap_msgs::GetStringAtPoint sap;
    sap.request.layer = "semantic";
    sap.request.location.header.frame_id = "map";
    sap.request.location.point.x = 3;
    sap.request.location.point.y = -2;
    ASSERT_EQ(getStringAtPointClient.call(sap), true) << "Failed to call service";
    ASSERT_EQ(sap.response.name, "table") << "Result should contain table";

    hypermap_msgs::GetStringsByArea sba;
    sba.request.layer = "semantic";
    sba.request.area.header.frame_id = "map";
    sba.request.area.polygon.points.push_back(createPoint32(3, -2));
    sba.request.area.polygon.points.push_back(createPoint32(3, -1));
    sba.request.area.polygon.points.push_back(createPoint32(2, -1));
    sba.request.area.polygon.points.push_back(createPoint32(2, -2));
    ASSERT_EQ(getStringsByAreaClient.call(sba), true) << "Failed to call service";
    const auto &sban = sba.response.names;
    ASSERT_EQ(sban.size(), 3) << "Result should return three objects in this area";
    ASSERT_NE(std::find(sban.begin(), sban.end(), "table"), sban.end()) << "Result should contain table";
    ASSERT_NE(std::find(sban.begin(), sban.end(), "tvmonitor"), sban.end()) << "Result should contain tvmonitor";
    ASSERT_NE(std::find(sban.begin(), sban.end(), "chair"), sban.end()) << "Result should contain chair";
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    package_path = ros::package::getPath("hypermap");

    //std::ofstream out_path("/home/tobias/path_tests_hmap.txt");
    //out_path << package_path;
    //out_path.close();

    nh = new ros::NodeHandle;
    return RUN_ALL_TESTS();
}
