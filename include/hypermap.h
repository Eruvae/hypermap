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

//using namespace libzippp;

class Hypermap
{
  std::vector<MapLayerBase> layers;
  //ZipArchive *mapFile;
  libzip::archive *mapFile;

  void loadMapFile(const char *path)
  {
      /*if (mapFile != 0)
          closeMapFile();

      mapFile = new ZipArchive(path);
      mapFile->open(ZipArchive::READ_ONLY);*/

      mapFile = new libzip::archive(path);
  }

  void closeMapFile()
  {
      /*mapFile->close();
      delete mapFile;
      mapFile = 0;*/
  }

public:
  ros::NodeHandle &nh;
  Hypermap(ros::NodeHandle &nh) : nh(nh), mapFile(0) {}

  void* getLayerFile(const char *fname, bool binary = false)
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
  }

  void testZip()
  {
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
