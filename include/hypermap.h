#ifndef HYPERMAP_H
#define HYPERMAP_H

#include "ros/ros.h"
#include "maplayerbase.h"
#include "occupancygridlayer.h"
#include "semanticlayer.h"
//#include "libzippp.h"
#include "zip.hpp"
#include <string>
#include <vector>
#include <map>

//using namespace libzippp;

class Hypermap
{
  std::map<std::string, size_t> strToInd;
  std::vector<MapLayerBase> layers;
  //ZipArchive *mapFile;
  //libzip::archive *mapFile;
  std::unique_ptr<libzip::archive> mapFile;

  void loadMapFile(const char *path)
  {
      /*if (mapFile != 0)
          closeMapFile();

      mapFile = new ZipArchive(path);
      mapFile->open(ZipArchive::READ_ONLY);*/


      /*if (mapFile != 0)
          closeMapFile();

      mapFile = new libzip::archive(path);*/

      try
      {
        mapFile.reset(new libzip::archive(path));
      }
      catch (std::runtime_error e)
      {
          ROS_ERROR("%s", e.what());
      }
  }

  void closeMapFile()
  {
      /*mapFile->close();
      delete mapFile;
      mapFile = 0;*/

      /*delete mapFile;
      mapFile = 0;*/
      mapFile.reset();
  }

public:
  ros::NodeHandle &nh;
  Hypermap(ros::NodeHandle &nh) : nh(nh)/*, mapFile(0)*/ {}

  std::unique_ptr<uint8_t[]> getLayerFile(const char *fname, bool binary = false)
  {
      /*if (mapFile == 0)
          return 0;

      if (mapFile->hasEntry(fname))
      {
          void *data;
          ZipEntry file = mapFile->getEntry(fname);
          if(binary)
              data = file.readAsBinary();
          else
              data = file.readAsBinary();

          return data;
      }
      return 0;*/

      if (mapFile.get() == nullptr)
      {
          ROS_ERROR("Map not opened");
          return nullptr;
      }

      try
      {
          int64_t index = mapFile->find(fname);
          libzip::stat_info stat = mapFile->stat(index);
          libzip::file file = mapFile->open(index);

          uint8_t *data = new uint8_t[stat.size];
          int size = file.read(data, stat.size);
          return std::unique_ptr<uint8_t[]>(data);
      }
      catch (std::runtime_error e)
      {
          ROS_ERROR("%s", e.what());
          return nullptr;
      }
  }

  void testZip()
  {
      try
      {
          //mapFile = new libzip::archive("testzip.zip");
          mapFile.reset(new libzip::archive("testzip.zip"));
      }
      catch (std::runtime_error e) {
          std::cout << e.what() << std::endl;
          return;
      }

      int64_t index = mapFile->find("readme.txt");
      libzip::stat_info stat = mapFile->stat(index);
      libzip::file file = mapFile->open(index);

      char *data = new char[stat.size + 1];
      int size = file.read(data, stat.size);
      data[size] = 0;

      std::cout << "Size readme (found, read): " << stat.size << ", " << size << std::endl;
      std::cout << data << std::endl;


      /*ZipArchive zf("testzip.zip");
      zf.open(ZipArchive::READ_ONLY);

      std::vector<ZipEntry> entries = zf.getEntries();
      std::vector<ZipEntry>::iterator it;
      for(it=entries.begin() ; it!=entries.end(); ++it) {
        ZipEntry entry = *it;
        std::string name = entry.getName();
        std::cout << name << std::endl;
        //int size = entry.getSize();

        //the length of binaryData will be size
        //void* binaryData = entry.readAsBinary();

        //the length of textData will be size
        std::string textData = entry.readAsText();

        //...
      }

      zf.close();*/
  }
};

#endif // HYPERMAP_H
