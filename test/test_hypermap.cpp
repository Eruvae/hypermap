#include <string>
//#include <fstream>
#include <gtest/gtest.h>
#include <ros/package.h>

#include "hypermap.h"

using hypermap::Hypermap;

std::string package_path;
ros::NodeHandle *nh;

TEST(HypermapTests, loadMapFile)
{
    Hypermap hm(*nh);
    hm.loadMapFile(package_path + "/test/hmap_example.hmap");
    ASSERT_EQ(2, hm.getLayerCnt());
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
