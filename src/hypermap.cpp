#include "hypermap.h"

#include <sstream>
#include <fstream>
#include <string>

#include <yaml-cpp/yaml.h>

#include "occupancygridlayer.h"
#include "semanticlayer.h"

#include <tf2/impl/convert.h>

namespace hypermap
{

bool Hypermap::transform(geometry_msgs::PointStamped &p, const std::string &target)
{
    try
    {
      //tfBuffer.transform(p, p, target, ros::Time(0), p.header.frame_id);
      tf2::doTransform(p, p, tfBuffer.lookupTransform(target, tf2::getFrameId(p), ros::Time(0), ros::Duration(0)));
      return true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR_STREAM("Cannot find transformation. What: " << ex.what());
      return false;
    }
}

bool Hypermap::transform(geometry_msgs::PolygonStamped &p, const std::string &target)
{
    try
    {
      //tfBuffer.transform(p, p, target, ros::Time(0), p.header.frame_id);
      tf2::doTransform(p, p, tfBuffer.lookupTransform(target, tf2::getFrameId(p), ros::Time(0), ros::Duration(0)));
      return true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR_STREAM("Cannot find transformation. What: " << ex.what());
      return false;
    }
}

bool Hypermap::transform(geometry_msgs::Point &p, const std::string &target, const std::string &fixed)
{
    try
    {
      //tfBuffer.transform(p, p, target, ros::Time(0), fixed);
      tf2::doTransform(p, p, tfBuffer.lookupTransform(target, fixed, ros::Time(0), ros::Duration(0)));
      return true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR_STREAM("Cannot find transformation. What: " << ex.what());
      return false;
    }
}

bool Hypermap::transform(geometry_msgs::Polygon &p, const std::string &target, const std::string &fixed)
{
    try
    {
      //tfBuffer.transform(p, p, target, ros::Time(0), fixed);
      tf2::doTransform(p, p, tfBuffer.lookupTransform(target, fixed, ros::Time(0), ros::Duration(0)));
      return true;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR_STREAM("Cannot find transformation. What: " << ex.what());
      return false;
    }
}

int Hypermap::getIntValue(const std::string &layer, geometry_msgs::PointStamped &p)
{
    MapLayerBase *lp = getLayer(layer);
    if (lp == 0)
        return -1;

    if (!transform(p, lp->getTfFrame()))
        return -1;

    return lp->getIntValue(p.point);
}

std::string Hypermap::getStringValue(const std::string &layer, geometry_msgs::PointStamped &p)
{
    MapLayerBase *lp = getLayer(layer);
    if (lp == 0)
        return "";

    if (!transform(p, lp->getTfFrame()))
        return "";

    return lp->getStringValue(p.point);
}

std::vector<std::pair<geometry_msgs::Point, int>> Hypermap::getIntValues(const std::string &layer, geometry_msgs::PolygonStamped &area)
{
    MapLayerBase *lp = getLayer(layer);
    if (lp == 0)
        return std::vector<std::pair<geometry_msgs::Point, int>>();

    if (!transform(area, lp->getTfFrame()))
        return std::vector<std::pair<geometry_msgs::Point, int>>();

    std::vector<std::pair<geometry_msgs::Point, int>> res = lp->getIntValues(area.polygon);
    for (auto &p : res)
        transform(p.first, getLayer(0)->getTfFrame(), lp->getTfFrame());

    return res;
}

std::vector<std::pair<geometry_msgs::Point, std::string>> Hypermap::getStringValues(const std::string &layer, geometry_msgs::PolygonStamped &area)
{
    MapLayerBase *lp = getLayer(layer);
    if (lp == 0)
        return std::vector<std::pair<geometry_msgs::Point, std::string>>();

    if (!transform(area, lp->getTfFrame()))
        return std::vector<std::pair<geometry_msgs::Point, std::string>>();

    std::vector<std::pair<geometry_msgs::Point, std::string>> res = lp->getStringValues(area.polygon);
    for (auto &p : res)
        transform(p.first, getLayer(0)->getTfFrame(), lp->getTfFrame());

    return res;
}

std::vector<geometry_msgs::Point> Hypermap::getCoords(const std::string &layer, int rep, geometry_msgs::PolygonStamped::Ptr area)
{
    MapLayerBase *lp = getLayer(layer);
    if (lp == 0)
        return std::vector<geometry_msgs::Point>();

    geometry_msgs::Polygon::Ptr area_ptr = 0;
    if (area)
    {
        if (!transform(*area, lp->getTfFrame()))
            return std::vector<geometry_msgs::Point>();

        area_ptr.reset(new geometry_msgs::Polygon);
        *area_ptr = area->polygon;
    }
    std::vector<geometry_msgs::Point> res = lp->getCoords(rep, area_ptr);
    for (geometry_msgs::Point &p : res)
        transform(p, getLayer(0)->getTfFrame(), lp->getTfFrame());

    return res;
}

std::vector<geometry_msgs::Point> Hypermap::getCoords(const std::string &layer, const std::string &rep, geometry_msgs::PolygonStamped::Ptr area)
{
    MapLayerBase *lp = getLayer(layer);
    if (lp == 0)
        return std::vector<geometry_msgs::Point>();

    geometry_msgs::Polygon::Ptr area_ptr = 0;
    if (area)
    {
        if (!transform(*area, lp->getTfFrame()))
            return std::vector<geometry_msgs::Point>();

        area_ptr.reset(new geometry_msgs::Polygon);
        *area_ptr = area->polygon;
    }
    std::vector<geometry_msgs::Point> res = lp->getCoords(rep, area_ptr);
    for (geometry_msgs::Point &p : res)
        transform(p, getLayer(0)->getTfFrame(), lp->getTfFrame());

    return res;
}

void Hypermap::clear()
{
    layers.clear();
    strToInd.clear();
    transforms.clear();
    closeMapFile();
}

void Hypermap::publishLayerData()
{
    for (const auto &layer : layers)
    {
        layer->publishData();
    }
    for (geometry_msgs::TransformStamped &transform : transforms)
    {
        transform.header.stamp = ros::Time::now();
        tfBroadcaster.sendTransform(transform);
    }
}

bool Hypermap::loadMapFile(const std::string &path)
{
    /*if (mapFile != 0)
        closeMapFile();

    mapFile = new ZipArchive(path);
    mapFile->open(ZipArchive::READ_ONLY);*/


    /*if (mapFile != 0)
        closeMapFile();

    mapFile = new libzip::archive(path);*/

    clear();

    try
    {
      mapFile.reset(new libzip::archive(path));
      std::istringstream conf(mapFile->read("hmap_config.yaml"));
      loadMapConfig(conf);
    }
    catch (const std::runtime_error &e)
    {
        ROS_ERROR_STREAM("Error loading file " << path << ": " << e.what());
        return false;
    }
    return true;
}

bool Hypermap::saveMapFile(const std::string &path)
{
    try
    {
      mapFile.reset(new libzip::archive(path, ZIP_CREATE | ZIP_TRUNCATE));

      for (const auto &layer : layers)
      {
          layer->saveMapData();
      }

      std::ostringstream toWrite;
      saveMapConfig(toWrite);
      std::cout << "Map config: " << toWrite.str() << std::endl;
      mapFile->add(libzip::source_buffer(toWrite.str()), "hmap_config.yaml", ZIP_FL_OVERWRITE);

      mapFile.reset();
    }
    catch (const std::runtime_error &e)
    {
        ROS_ERROR_STREAM("Error storing file " << path << ": " << e.what());
        return false;
    }
    return true;
}

void Hypermap::loadMapConfig(std::istream &data)
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
        bool subscribe_mode = false, enable_update = true;
        if (layer["subscribe_mode"])
            subscribe_mode = layer["subscribe_mode"].as<bool>();
        if (layer["enable_update"])
            enable_update = layer["enable_update"].as<bool>();

        std::cout << "File load: " << load_file << std::endl;

        if (class_name == "OccupancyGridLayer")
        {
            layers.push_back(std::make_unique<OccupancyGridLayer>(this, name, frame_id, subscribe_mode, enable_update));
        }
        else if (class_name == "SemanticLayer")
        {
            layers.push_back(std::make_unique<SemanticLayer>(this, name, frame_id, subscribe_mode, enable_update));
        }
        else
        {
            throw std::runtime_error("Error loading map: Layer class not recognized!");
        }
        strToInd[name] = ind;
        if (load_file)
        {
            std::cout << "Loading Map" << std::endl;
            layers[ind]->loadMapData(layer["file"].as<std::string>());
        }
    }
    if (conf["transforms"])
    {
        for(const YAML::Node &transform : conf["transforms"])
        {
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = transform["frame_id"].as<std::string>();
            transformStamped.child_frame_id = transform["child_frame_id"].as<std::string>();
            const YAML::Node &transformPose= transform["transform_pose"];
            transformStamped.transform.translation.x = transformPose[0].as<double>();
            transformStamped.transform.translation.y = transformPose[1].as<double>();
            transformStamped.transform.translation.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, transformPose[2].as<double>());
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();

            transforms.push_back(transformStamped);
        }
    }
    publishLayerData();
}

void Hypermap::saveMapConfig(std::ostream &out)
{
    YAML::Node conf;
    for (const auto &layer : layers)
    {
        YAML::Node l;
        l["name"] = layer->getName();
        l["frame_id"] = layer->getTfFrame();
        l["load_file"] = "true";
        l["file"] = layer->getFileName();

        if (typeid(*layer) == typeid(OccupancyGridLayer))
        {
            l["class"] = "OccupancyGridLayer";
        }
        else if (typeid(*layer) == typeid(SemanticLayer))
        {
            l["class"] = "SemanticLayer";
        }
        else
        {
            ROS_ERROR("Error saving map: Layer class not recognized!");
            return;
        }

        conf["layers"].push_back(l);
    }
    out << conf;
}

/*std::string Hypermap::getLayerFile(const char *fname)
{
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
}*/

std::string Hypermap::getLayerFile(const std::string &fname)
{
    if (mapFile.get() == nullptr)
    {
        ROS_ERROR("Map not opened");
        return "";
    }

    try
    {
        //int64_t index = mapFile->find(fname);
        //libzip::stat_info stat = mapFile->stat(index);
        //libzip::file file = mapFile->open(index);

        return mapFile->read(fname); //file.read(stat.size);
    }
    catch (std::runtime_error e)
    {
        ROS_ERROR("%s", e.what());
        return "";
    }
}

bool Hypermap::getLayerFile(const std::string &fname, std::function<bool(std::istream&)> getter)
{
    if (mapFile.get() == nullptr) // No map file, read from file system
    {
        //ROS_ERROR("Map not opened");
        //return false;
        try
        {
            std::ifstream file(fname);
            if (!file.good())
            {
                ROS_ERROR_STREAM("File " << fname << " could not be opened");
                return false;
            }
            return getter(file);
        }
        catch (const std::runtime_error &e)
        {
            ROS_ERROR("%s", e.what());
            return false;
        }
    }
    else
    {
        try
        {
            std::string data = mapFile->read(fname);
            std::istringstream str(data);
            return getter(str);
        }
        catch (const std::runtime_error &e)
        {
            ROS_ERROR("%s", e.what());
            return false;
        }
    }
}

bool Hypermap::getLayerFile(const std::string &fname, std::function<bool(const std::string&)> getter)
{
    if (mapFile.get() == nullptr)
    {
        //ROS_ERROR("Map not opened");
        //return false;
        try
        {
            std::ifstream file(fname, std::ios::binary);
            if (!file.good())
            {
                ROS_ERROR_STREAM("File " << fname << " could not be opened");
                return false;
            }
            file.seekg(0, file.end);
            int length = file.tellg();
            file.seekg(0, file.beg);

            std::string data;
            data.resize(length);

            file.read (&data[0], length);

            return getter(data);
        }
        catch (const std::runtime_error &e)
        {
            ROS_ERROR("%s", e.what());
            return false;
        }
    }
    else
    {
        try
        {
            std::string data = mapFile->read(fname);
            return getter(data);
        }
        catch (const std::runtime_error &e)
        {
            ROS_ERROR("%s", e.what());
            return false;
        }
    }
}

bool Hypermap::putLayerFile(const std::string &fname, const std::string &data)
{
    if (mapFile.get() == nullptr)
    {
        ROS_ERROR("Map not opened");
        return false;
    }

    try
    {
        mapFile->add(libzip::source_buffer(data), fname, ZIP_FL_OVERWRITE);
    }
    catch (std::runtime_error e)
    {
        ROS_ERROR("%s", e.what());
        return false;
    }
    return true;
}

bool Hypermap::putLayerFile(const std::string &fname, std::function<bool(std::ostream&)> putter)
{
    if (mapFile.get() == nullptr)
    {
        ROS_ERROR("Map not opened");
        return false;
    }

    std::ostringstream str;

    if (putter(str))
    {
        try
        {
            mapFile->add(libzip::source_buffer(str.str()), fname, ZIP_FL_OVERWRITE);
        }
        catch(std::runtime_error e)
        {
            ROS_ERROR("%s", e.what());
            return false;
        }
        return true;
    }
    return false;
}

bool Hypermap::putLayerFile(const std::string &fname, std::function<std::string()> putter)
{
    if (mapFile.get() == nullptr)
    {
        ROS_ERROR("Map not opened");
        return false;
    }

    try
    {
        mapFile->add(libzip::source_buffer(putter()), fname, ZIP_FL_OVERWRITE);
    }
    catch (std::runtime_error e)
    {
        ROS_ERROR("%s", e.what());
        return false;
    }
    return true;
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

}
