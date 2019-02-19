#include "hypermap.h"

void Hypermap::loadMapFile(const char *path)
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

void Hypermap::closeMapFile()
{
    /*mapFile->close();
    delete mapFile;
    mapFile = 0;*/

    /*delete mapFile;
    mapFile = 0;*/
    mapFile.reset();
}

void Hypermap::loadMapConfig(const std::string &data)
{
    YAML::Node conf = YAML::Load(data);
    std::cout << "Data loaded" << std::endl;
    for (const YAML::Node &layer : conf["layers"])
    {
        std::cout << "Layer config found" << std::endl;
        std::string class_name = layer["class"].as<std::string>();
        std::string frame_id = layer["frame_id"].as<std::string>();
        std::string name = layer["name"].as<std::string>();
        size_t ind = layers.size();
        bool load_file = layer["load_file"].as<bool>();

        std::cout << "File load: " << load_file << std::endl;

        if (class_name == "OccupancyGridLayer")
        {
            layers.push_back(std::make_unique<OccupancyGridLayer>(this));
        }
        else if (class_name == "SemanticLayer")
        {
            layers.push_back(std::make_unique<SemanticLayer>(this));
        }
        else
        {
            ROS_ERROR("Error loading map: Layer class not recognized!");
            return;
        }
        strToInd[name] = ind;
        if (load_file)
        {
            std::cout << "Loading Map" << std::endl;
            layers[ind]->loadMapData(layer["file"].as<std::string>());
        }
    }
}

void Hypermap::transformPoint(geometry_msgs::Point &p, const std::string &origin, const std::string &target)
{

}

void Hypermap::transformPolygon(geometry_msgs::Polygon &p, const std::string &origin, const std::string &target)
{

}

std::string Hypermap::getLayerFile(const char *fname)
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
        return "";
    }

    try
    {
        int64_t index = mapFile->find(fname);
        libzip::stat_info stat = mapFile->stat(index);
        libzip::file file = mapFile->open(index);

        return file.read(stat.size);
    }
    catch (std::runtime_error e)
    {
        ROS_ERROR("%s", e.what());
        return "";
    }
}

void Hypermap::testZip()
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
