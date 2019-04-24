#include "occupancygridlayer.h"

#include <functional>

#include <yaml-cpp/yaml.h>
//#include <SDL2/SDL_image.h>

#include <png++/image.hpp>
//#include <png++/solid_pixel_buffer.hpp>

/*#include <boost/gil/image.hpp>
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/dynamic_image/any_image.hpp>
#include <boost/gil/extension/io/dynamic_io.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>
#include <boost/gil/extension/io/jpeg_dynamic_io.hpp>
#include <boost/gil/image_view.hpp>
#include <boost/gil/utilities.hpp>
#include <boost/gil/gray.hpp>*/

#include "hypermap.h"

#include "stb_image.h"

namespace hypermap
{

OccupancyGridLayer::OccupancyGridLayer(Hypermap *parent, const std::string &name, const std::string &tfFrame, bool subscribe_mode, bool enable_update)
    : MapLayerBase(parent, name, tfFrame, subscribe_mode, enable_update)
{
    if (parent == 0)
        return;

    mapPub = parent->nh.advertise<nav_msgs::OccupancyGrid>(name + "_map", 1);
    if (!mapPub)
        ROS_ERROR("Map publisher not initialized!");

    mapMetaPub = parent->nh.advertise<nav_msgs::MapMetaData>(name + "_map_metadata", 1);
    if (!mapMetaPub)
        ROS_ERROR("Map meta publisher not initialized!");

    mapService = parent->nh.advertiseService(name + "_static_map", &OccupancyGridLayer::mapCallback, this);

    if (subscribe_mode)
    {
        mapSub = parent->nh.subscribe("/map", 10, &OccupancyGridLayer::updateMap, this);
        mapMetaSub = parent->nh.subscribe("/map_metadata", 10, &OccupancyGridLayer::updateMapMeta, this);
    }
}

int OccupancyGridLayer::getIntValue(const geometry_msgs::Point &p)
{
    return getGridData(getPointIndex(p));
}

std::string OccupancyGridLayer::getStringValue(const geometry_msgs::Point &p)
{
    return getGridString(getPointIndex(p));

}

std::vector<std::pair<geometry_msgs::Point, std::string>> OccupancyGridLayer::getStringValues(const geometry_msgs::Polygon &area)
{
  std::vector<MapIndex> inds = getGridIndices(area);
  std::vector<std::pair<geometry_msgs::Point, std::string>> res;
  for (const MapIndex &ind : inds)
  {
      res.push_back(std::make_pair(getCoordinatesMsg(ind), getGridString(ind)));
  }
  return res;
}

std::vector<std::pair<geometry_msgs::Point, int>> OccupancyGridLayer::getIntValues(const geometry_msgs::Polygon &area)
{
    std::vector<MapIndex> inds = getGridIndices(area);
    std::vector<std::pair<geometry_msgs::Point, int>> res;
    for (const MapIndex &ind : inds)
    {
        res.push_back(std::make_pair(getCoordinatesMsg(ind), getGridData(ind)));
    }
    return res;
}

std::vector<geometry_msgs::Point> OccupancyGridLayer::getCoords(const std::string &rep, geometry_msgs::Polygon::ConstPtr area)
{
    std::vector<geometry_msgs::Point> res;
    if(area)
    {
        std::vector<MapIndex> inds = getGridIndices(*area);
        for (const MapIndex &ind : inds)
        {
            if (rep == getGridString(ind))
            {
                res.push_back(getCoordinatesMsg(ind));
            }
        }
    }
    else
    {
        for (int i = 0; i < map.info.width; i++)
        {
            for (int j = 0; j < map.info.height; j++)
            {
                MapIndex ind = {i, j};
                if (rep == getGridString(ind))
                {
                    res.push_back(getCoordinatesMsg(ind));
                }
            }
        }
    }
    return res;
}

std::vector<geometry_msgs::Point> OccupancyGridLayer::getCoords(int rep, geometry_msgs::Polygon::ConstPtr area)
{
    std::vector<geometry_msgs::Point> res;
    if(area)
    {
        std::vector<MapIndex> inds = getGridIndices(*area);
        for (const MapIndex &ind : inds)
        {
            if (rep == getGridData(ind))
            {
                res.push_back(getCoordinatesMsg(ind));
            }
        }
    }
    else
    {
        for (int i = 0; i < map.info.width; i++)
        {
            for (int j = 0; j < map.info.height; j++)
            {
                MapIndex ind = {i, j};
                if (rep == getGridData(ind))
                {
                    res.push_back(getCoordinatesMsg(ind));
                }
            }
        }
    }
    return res;
}

std::string OccupancyGridLayer::getGridString(const MapIndex &ind)
{
    int8_t data = getGridData(ind);
    if (data == 0)
        return "Free";
    else if (data == 100)
        return "Occupied";
    else
        return "Unknown";
}

void OccupancyGridLayer::setSubscribeMode(bool mode)
{
    if (subscribe_mode == true && mode == false)
    {
        mapSub.shutdown();
        mapMetaSub.shutdown();
    }
    else if (subscribe_mode = false && mode == true)
    {
        mapSub = parent->nh.subscribe("/map", 100, &OccupancyGridLayer::updateMap, this);
        mapMetaSub = parent->nh.subscribe("/map_metadata", 100, &OccupancyGridLayer::updateMapMeta, this);
    }
    subscribe_mode = mode;
}

bool OccupancyGridLayer::mapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
{
    res = resp;
    return true;
}

void OccupancyGridLayer::loadMapData(const std::string &file_name)
{
    if (parent == 0)
        return;

    /* name = file_name;
    std::string meta_file = parent->getLayerFile(name + ".yaml");
    std::istringstream meta_stream(meta_file);
    if (!loadMapMeta(meta_stream))
    {
        ROS_ERROR("Map meta data corrupted, map not loaded.");
        return;
    }
    std::string map_file = parent->getLayerFile(mapfname);
    loadMap(map_file);*/

    this->file_name = file_name;

    if (!parent->getLayerFile(file_name + ".yaml", std::bind(&OccupancyGridLayer::loadMapMeta, this, std::placeholders::_1)))
    {
        ROS_ERROR("Map meta data corrupted, map not loaded.");
        return;
    }
    if (!parent->getLayerFile(fileData.image, std::bind(&OccupancyGridLayer::loadMap, this, std::placeholders::_1)))
    {
        ROS_ERROR("Map image data corrupted, map not loaded.");
        return;
    }

    publishData();
}

void OccupancyGridLayer::saveMapData()
{
    if (parent == 0)
        return;

    /* mapfname = name + "*.pgm";
    std::ostringstream map_meta;
    saveMapMeta(map_meta);
    parent->putLayerFile(name + ".yaml", map_meta.str());

    std::ostringstream map_file;
    saveMap(map_file);
    parent->putLayerFile(mapfname, map_file.str());*/

    //mapfname = file_name + ".png";

    fileData.image = file_name + ".png";
    fileData.resolution = map.info.resolution;
    fileData.origin[0] = map.info.origin.position.x;
    fileData.origin[1] = map.info.origin.position.y;
    fileData.origin[2] = getYaw();
    fileData.occupied_thresh = 1.0;
    fileData.free_thresh = 0.0;
    fileData.negate = false;
    fileData.mode = SCALE;

    parent->putLayerFile(file_name + ".yaml", std::bind(&OccupancyGridLayer::saveMapMeta, this, std::placeholders::_1));
    parent->putLayerFile(fileData.image, std::bind(&OccupancyGridLayer::saveMap, this, std::placeholders::_1));
}

void OccupancyGridLayer::publishData()
{
    if (subscribe_mode)
        return;

    mapPub.publish(map);
    mapMetaPub.publish(map.info);
    ros::spinOnce();
    ROS_INFO("Occupancy Layer map published");
}

// Functions loadMapMeta and loadMap based on map_server node
// https://github.com/ros-planning/navigation.git
// Original author: Brian Gerkey

/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

bool OccupancyGridLayer::loadMapMeta(std::istream &in)
{
    YAML::Node doc = YAML::Load(in);
    try {
        fileData.resolution = doc["resolution"].as<double>();
    } catch (const YAML::Exception &) {
        ROS_ERROR("The map does not contain a resolution tag or it is invalid.");
        return false;
    }
    try {
        fileData.negate = doc["negate"].as<int>();
    } catch (const YAML::Exception &) {
        ROS_ERROR("The map does not contain a negate tag or it is invalid.");
        return false;
    }
    try {
        fileData.occupied_thresh = doc["occupied_thresh"].as<double>();
    } catch (YAML::Exception &) {
        ROS_ERROR("The map does not contain an occupied_thresh tag or it is invalid.");
        return false;
    }
    try {
        fileData.free_thresh = doc["free_thresh"].as<double>();
    } catch (YAML::Exception &) {
        ROS_ERROR("The map does not contain a free_thresh tag or it is invalid.");
        return false;
    }
    try {
        std::string modeS = doc["mode"].as<std::string>();

        if(modeS=="trinary")
            fileData.mode = TRINARY;
        else if(modeS=="scale")
            fileData.mode = SCALE;
        else if(modeS=="raw")
            fileData.mode = RAW;
        else{
            ROS_ERROR("Invalid mode tag \"%s\".", modeS.c_str());
            return false;
        }
    } catch (YAML::Exception &) {
        ROS_DEBUG("The map does not contain a mode tag or it is invalid... assuming Trinary");
        fileData.mode = TRINARY;
    }
    try {
        fileData.origin[0] = doc["origin"][0].as<double>();
        fileData.origin[1] = doc["origin"][1].as<double>();
        fileData.origin[2] = doc["origin"][2].as<double>();
    } catch (YAML::Exception &) {
        ROS_ERROR("The map does not contain an origin tag or it is invalid.");
        return false;
    }
    try {
        fileData.image = doc["image"].as<std::string>();
        // TODO: make this path-handling more robust
        if(fileData.image.size() == 0)
        {
            ROS_ERROR("The image tag cannot be an empty string.");
            return false;
        }
    } catch (YAML::Exception &) {
        ROS_ERROR("The map does not contain an image tag or it is invalid.");
        return false;
    }
    return true;
}

bool OccupancyGridLayer::loadMap(const std::string &data)
{
    int w, h;
    int channels_in_file;
    int channels = STBI_grey_alpha; // read in grey-alpha mode (2 channels)
    uint8_t *pixels = stbi_load_from_memory((const uint8_t*) &data[0], data.size(), &w, &h, &channels_in_file, channels);
    std::cout << "Channels in file: " << channels_in_file << std::endl;
    if(pixels == NULL)
    {
      ROS_ERROR("failed to open image: %s", stbi_failure_reason());
      return false;
    }

    // Copy the image data into the map structure
    map.info.width = w;
    map.info.height = h;
    map.info.resolution = fileData.resolution;
    map.info.origin.position.x = fileData.origin[0];
    map.info.origin.position.y = fileData.origin[1];
    map.info.origin.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, fileData.origin[2]);
    map.info.origin.orientation.x = q.x();
    map.info.origin.orientation.y = q.y();
    map.info.origin.orientation.z = q.z();
    map.info.origin.orientation.w = q.w();

    // Allocate space to hold the data
    map.data.resize(map.info.width * map.info.height);

    for(size_t j = 0; j < map.info.height; j++)
    {
      for (size_t i = 0; i < map.info.width; i++)
      {
          uint8_t *p = pixels + j*channels*w + i*channels;
          uint8_t color = p[0];
          uint8_t alpha = p[1];

          if(fileData.negate)
            color = 255 - color;

          int8_t value;

          if(fileData.mode==RAW){
              value = color;
              map.data[getDataIndex(i, map.info.height - j - 1)] = value;
              continue;
          }

          // If negate is true, we consider blacker pixels free, and whiter
          // pixels occupied.  Otherwise, it's vice versa.
          double occ = (255 - color) / 255.0;

          // Apply thresholds to RGB means to determine occupancy values for
          // map.  Note that we invert the graphics-ordering of the pixels to
          // produce a map with cell (0,0) in the lower-left corner.
          if (alpha < 255)
            value = -1;
          else if(occ > fileData.occupied_thresh)
            value = 100;
          else if(occ < fileData.free_thresh)
            value = 0;
          else if(fileData.mode==TRINARY)
            value = -1;
          else {
            double ratio = (occ - fileData.free_thresh) / (fileData.occupied_thresh - fileData.free_thresh);
            value = (uint8_t) std::round(100 * ratio);
          }

          map.data[getDataIndex(i, map.info.height - j - 1)] = value;
      }
    }
    stbi_image_free(pixels);
    return true;
}

/*bool OccupancyGridLayer::loadMap(const std::string &data)
{
    SDL_Surface* img;

    unsigned char* pixels;
    unsigned char* p;
    unsigned char value;
    int rowstride, n_channels, avg_channels;
    unsigned int i,j;
    int k;
    double occ;
    int alpha;
    int color_sum;
    double color_avg;

    // Load the image using SDL.  If we get NULL back, the image load failed.

    SDL_RWops* instr = SDL_RWFromConstMem(&data[0], data.size());
    if(!(img = IMG_Load_RW(instr, true)))
    {
      ROS_ERROR("failed to open image: %s", IMG_GetError());
      return false;
    }

    // Copy the image data into the map structure
    map.info.width = img->w;
    map.info.height = img->h;
    map.info.resolution = fileData.resolution;
    map.info.origin.position.x = fileData.origin[0];
    map.info.origin.position.y = fileData.origin[1];
    map.info.origin.position.z = 0.0;
    tf2::Quaternion q;
    // setEulerZYX(yaw, pitch, roll)
    // q.setEulerZYX(*(origin+2), 0, 0);
    q.setRPY(0, 0, fileData.origin[2]);
    map.info.origin.orientation.x = q.x();
    map.info.origin.orientation.y = q.y();
    map.info.origin.orientation.z = q.z();
    map.info.origin.orientation.w = q.w();

    // Allocate space to hold the data
    map.data.resize(map.info.width * map.info.height);

    // Get values that we'll need to iterate through the pixels
    rowstride = img->pitch;
    n_channels = img->format->BytesPerPixel;

    std::cout << "Bytes: " << (int)img->format->BytesPerPixel << ", Masks: " << img->format->Rmask << "; " << img->format->Gmask << "; " << img->format->Bmask << "; " << img->format->Amask << std::endl;

    // NOTE: Trinary mode still overrides here to preserve existing behavior.
    // Alpha will be averaged in with color channels when using trinary mode.
    if (fileData.mode==TRINARY || !img->format->Amask)
      avg_channels = n_channels;
    else
      avg_channels = n_channels - 1;

    // Copy pixel data into the map structure
    pixels = (unsigned char*)(img->pixels);
    for(j = 0; j < map.info.height; j++)
    {
      for (i = 0; i < map.info.width; i++)
      {
        // Compute mean of RGB for this pixel
        p = pixels + j*rowstride + i*n_channels;
        color_sum = 0;
        for(k=0;k<avg_channels;k++)
          color_sum += *(p + (k));
        color_avg = color_sum / (double)avg_channels;

        if (n_channels < 4)
            alpha = 255;
        else
            alpha = *(p+n_channels-1);

        if(fileData.negate)
          color_avg = 255 - color_avg;

        if(fileData.mode==RAW){
            value = color_avg;
            map.data[getDataIndex(i, map.info.height - j - 1)] = value;
            continue;
        }


        // If negate is true, we consider blacker pixels free, and whiter
        // pixels occupied.  Otherwise, it's vice versa.
        occ = (255 - color_avg) / 255.0;

        // Apply thresholds to RGB means to determine occupancy values for
        // map.  Note that we invert the graphics-ordering of the pixels to
        // produce a map with cell (0,0) in the lower-left corner.
        if (alpha < 255)
          value = -1;
        else if(occ > fileData.occupied_thresh)
          value = 100;
        else if(occ < fileData.free_thresh)
          value = 0;
        else if(fileData.mode==TRINARY)
          value = -1;
        else {
          double ratio = (occ - fileData.free_thresh) / (fileData.occupied_thresh - fileData.free_thresh);
          value = (uint8_t) std::round(100 * ratio);
        }

        map.data[getDataIndex(i, map.info.height - j - 1)] = value;
      }
    }

    SDL_FreeSurface(img);
    return true;
}*/

bool OccupancyGridLayer::saveMapMeta(std::ostream &out)
{
    std::string modeS;
    if (fileData.mode == TRINARY)
        modeS = "trinary";
    else if (fileData.mode == SCALE)
        modeS = "scale";
    else
        modeS = "raw";

    out << "image: " << fileData.image << std::endl << "resolution: " << fileData.resolution << std::endl
        << "origin: [" << fileData.origin[0] << ", " << fileData.origin[1] << ", " << fileData.origin[2] << "]" << std::endl
        << "negate: " << fileData.negate << std::endl << "occupied_thresh: " << fileData.occupied_thresh << std::endl << "free_thresh: " << fileData.free_thresh << std::endl
        << "mode: " << modeS << std::endl << std::endl;
    return true;
}

bool OccupancyGridLayer::saveMap(std::ostream &out)
{
    /*out << "P5" << std::endl; // binary portable greymap
    out << "# CREATOR: hypermap; " << map.info.resolution << " m/pix." << std::endl;
    out << map.info.width << std::endl << map.info.height << std::endl << "255" << std::endl;
    for(unsigned int y = 0; y < map.info.height; y++)
    {
        for(unsigned int x = 0; x < map.info.width; x++)
        {
            unsigned int i = x + (map.info.height - y - 1) * map.info.width;
            if (map.data[i] >= 0 && map.data[i] <= (int)(free_th * 100))
            { // [0,free)
                out.put(254);
            }
            else if (map.data[i] >= (int)(occ_th * 100))
            { // (occ,255]
                out.put(000);
            }
            else
            { //occ [0.25,0.65]
                out.put(205);
            }
        }
    }*/

    png::image<png::ga_pixel> out_img(map.info.width, map.info.height);
    for(unsigned int y = 0; y < map.info.height; y++)
    {
        for(unsigned int x = 0; x < map.info.width; x++)
        {
            unsigned int i = x + (map.info.height - y - 1) * map.info.width;
            if (map.data[i] < 0) // unknown
            {
                out_img[y][x] = png::ga_pixel(127, 0);
            }
            else
            {
                //double dval = (100 - map.data[i]) / 100.0 * 255.0;
                //uint8_t val = std::round(std::max(dval, 0.0));
                double ratio = std::max(0.0, std::min(map.data[i] / 100.0, 1.0));
                double occ = ratio * (fileData.occupied_thresh - fileData.free_thresh) + fileData.free_thresh;
                uint8_t val = (uint8_t) std::round(255.0 - occ * 255.0);
                if (fileData.negate)
                    val = 255 - val;

                //std::cout << "Map: " << (int)map.data[i] << "; Ratio: " << ratio << "; Occ: " << occ << "; val: " << (int)val << std::endl;
                out_img[y][x] = png::ga_pixel(val, 255);
            }
        }
    }
    out_img.write_stream(out);
    return true;
}

}
