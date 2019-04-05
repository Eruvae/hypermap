#include "semanticlayer.h"

#include <functional>

#include <boost/geometry/io/wkt/wkt.hpp>
#include <yaml-cpp/yaml.h>

#include "hypermap.h"

namespace hypermap
{

SemanticLayer::SemanticLayer(Hypermap *parent, const std::string &name, const std::string &tfFrame, bool subscribe_mode, bool enable_update)
    : MapLayerBase(parent, name, tfFrame, subscribe_mode, enable_update), next_index(0)
{
    if (parent == 0)
        return;

    semmapPub = parent->nh.advertise<hypermap_msgs::SemanticMap>(name + "_semmap", 1);
    if (!semmapPub)
        ROS_ERROR("Semantic map publisher not initialized!");
}

/*SemanticLayer::SemanticLayer() : MapLayerBase("map")
{

}*/

void SemanticLayer::updateMap(const hypermap_msgs::SemanticMapUpdate::ConstPtr update)
{
    for (const hypermap_msgs::SemanticObject &obj : update->to_add)
    {
        addObject(createSemanticObjFromMessage(obj));
    }
    for (size_t id : update->to_modify)
    {
        updateObject(id, createSemanticObjFromMessage(update->updates[id]));
    }
    for (size_t id : update->to_delete)
    {
        removeObject(id);
    }
    rebuildMapMsg();
    publishData();
}

void SemanticLayer::clear()
{
    objectList.clear();
    objectMap.clear();
    objectRtree.clear();
    mapMsg.objects.clear();
    next_index = 0;
}

void SemanticLayer::rebuildMapMsg()
{
    mapMsg.objects.clear();
    for (const auto &val : objectList)
    {
        mapMsg.objects.push_back(semanticObjectToMsg(val.second));
    }
}

void SemanticLayer::addObject(const SemanticObject &newObject)
{
    objectList[next_index] = newObject;
    objectMap[newObject.name].insert(next_index);
    objectRtree.insert(std::make_pair(newObject.bounding_box, next_index));
    mapMsg.objects.push_back(semanticObjectToMsg(newObject));
    next_index++;
}

bool SemanticLayer::updateObject(size_t id, const SemanticObject &newObject)
{
    SemanticObject &object = objectList[id];
    if (newObject.name != object.name) // update keyword map
    {
        objectMap[object.name].erase(id);
        object.name = newObject.name;
        objectMap[object.name].insert(id);
    }
    if (!bg::equals(newObject.shape, object.shape))
    {
        object.shape = newObject.shape;
        if (!bg::equals(newObject.bounding_box, object.bounding_box)) // update rtree
        {
            objectRtree.remove(std::make_pair(object.bounding_box, id));
            object.bounding_box = newObject.bounding_box;
            objectRtree.insert(std::make_pair(object.bounding_box, id));
        }
        object.position = newObject.position;
    }
    // TODO: consider updating object in map message to avoid rebuilding
    return true;
}

bool SemanticLayer::removeObject(size_t id)
{
    SemanticObject &object = objectList[id];
    objectMap[object.name].erase(id);
    objectRtree.remove(std::make_pair(object.bounding_box, id));
    objectList.erase(id);
    // TODO: consider removing object from message to avoid rebuilding
    return true;
}

hypermap_msgs::SemanticObject SemanticLayer::semanticObjectToMsg(const SemanticObject &obj)
{
    hypermap_msgs::SemanticObject msg;
    msg.name = obj.name;
    msg.shape = boostToPolygonMsg(obj.shape);
    msg.position = boostToPointMsg(obj.position);
    msg.tags = obj.tags;
    msg.confidence = obj.confidence;
    return msg;
}

SemanticLayer::SemanticObject SemanticLayer::createSemanticObjFromMessage(const hypermap_msgs::SemanticObject &msg)
{
    SemanticObject obj;
    obj.name = msg.name;
    obj.shape = polygonMsgToBoost(msg.shape);
    obj.bounding_box = bg::return_envelope<box>(obj.shape);
    bg::centroid(obj.shape, obj.position);
    obj.tags = msg.tags;
    obj.confidence = msg.confidence;
    return obj;
}

int SemanticLayer::getIntValue(const geometry_msgs::Point &p)
{
    std::set<size_t> objects = getObjectsAt(pointMsgToBoost(p));
    if (objects.size() == 0)
        return -1;

    return *objects.begin(); // TODO: which object? layer/height
}

std::string SemanticLayer::getStringValue(const geometry_msgs::Point &p)
{
    std::set<size_t> objects = getObjectsAt(pointMsgToBoost(p));
    if (objects.size() == 0)
        return "Emtpy";

    return objectList[*objects.begin()].name; // TODO: see getIntValue
}

std::vector<std::pair<geometry_msgs::Point, int>> SemanticLayer::getIntValues(const geometry_msgs::Polygon &area)
{
    std::set<size_t> qres = getObjectsInRange(polygonMsgToBoost(area));
    std::vector<std::pair<geometry_msgs::Point, int>> res;
    for (size_t i : qres)
    {
        const SemanticObject &obj = objectList[i];
        /*point center;
        bg::centroid(obj.shape, center);
        res.push_back(std::make_pair(boostToPointMsg(center), i));*/
        res.push_back(std::make_pair(boostToPointMsg(obj.position), i));
    }
    return res;
}

std::vector<std::pair<geometry_msgs::Point, std::string>> SemanticLayer::getStringValues(const geometry_msgs::Polygon &area)
{
    std::set<size_t> qres = getObjectsInRange(polygonMsgToBoost(area));
    std::vector<std::pair<geometry_msgs::Point, std::string>> res;
    for (size_t i : qres)
    {
        const SemanticObject &obj = objectList[i];
        /*point center;
        bg::centroid(obj.shape, center);
        res.push_back(std::make_pair(boostToPointMsg(center), obj.name));*/
        res.push_back(std::make_pair(boostToPointMsg(obj.position), obj.name));
    }
    return res;
}

std::vector<geometry_msgs::Point> SemanticLayer::getCoords(int rep, geometry_msgs::Polygon::ConstPtr area)
{
    std::vector<geometry_msgs::Point> res;
    std::map<size_t, SemanticObject>::const_iterator it = objectList.find(rep);
    if (it != objectList.end())
    {
        const SemanticObject &obj = it->second;
        /*point center;
        bg::centroid(obj.shape, center);
        res.push_back(boostToPointMsg(center));*/
        res.push_back(boostToPointMsg(obj.position));
    }
    return res;
}

std::vector<geometry_msgs::Point> SemanticLayer::getCoords(const std::string &rep, geometry_msgs::Polygon::ConstPtr area)
{
    std::set<std::size_t> qres;
    std::vector<geometry_msgs::Point> res;
    if(area)
        qres = getObjectsByNameInRange(rep, polygonMsgToBoost(*area));
    else
        qres = getObjectsByName(rep);

    for (size_t i : qres)
    {
        const SemanticObject &obj = objectList[i];
        /*point center;
        bg::centroid(obj.shape, center);
        res.push_back(boostToPointMsg(center));*/
        res.push_back(boostToPointMsg(obj.position));
    }
    return res;
}

std::set<size_t> SemanticLayer::getObjectsAt(const point &p)
{
    std::vector<rtree_entry> result_obj;
    std::set<size_t> result;
    objectRtree.query(bgi::covers(p), std::back_inserter(result_obj));
    ROS_INFO_STREAM("Rtree size: " << objectRtree.size() << "; Found objects: " << result_obj.size());
    for (rtree_entry val : result_obj)
    {
        const SemanticObject &foundObject = objectList[val.second];
        ROS_INFO_STREAM("Found object: " << foundObject.name);
        if (bg::covered_by(p, foundObject.shape))
        {
            ROS_INFO("Point covered");
            result.insert(val.second);
        }
        else
        {
            ROS_INFO("Point not covered");
        }
    }
    return result;
}

std::set<size_t> SemanticLayer::getObjectsInRange(double xmin, double ymin, double xmax, double ymax)
{
    return getObjectsInRange(point(xmin, ymin), point(xmax, ymax));
}

std::set<size_t> SemanticLayer::getObjectsInRange(const point &minCorner, const point &maxCorner)
{
    box query_box(minCorner, maxCorner);
    std::vector<rtree_entry> result_obj;
    std::set<size_t> result;
    objectRtree.query(bgi::intersects(query_box), std::back_inserter(result_obj));
    for (rtree_entry val : result_obj)
    {
        const SemanticObject &foundObject = objectList[val.second];
        if (bg::intersects(query_box, foundObject.shape))
            result.insert(val.second);
    }
    return result;
}

std::set<size_t> SemanticLayer::getObjectsInRange(const polygon &pg)
{
    std::vector<rtree_entry> result_obj;
    std::set<size_t> result;
    objectRtree.query(bgi::intersects(pg), std::back_inserter(result_obj));
    for (rtree_entry val : result_obj)
    {
        const SemanticObject &foundObject = objectList[val.second];
        if (bg::intersects(pg, foundObject.shape))
            result.insert(val.second);
    }
    return result;
}

bool SemanticLayer::getSemanticByArea(hypermap_msgs::GetSemanticByArea::Request &req, hypermap_msgs::GetSemanticByArea::Response &res)
{
    auto qres = getObjectsInRange(polygonMsgToBoost(req.area.polygon));
    for (size_t i : qres)
    {
        res.objects.push_back(semanticObjectToMsg(objectList[i]));
    }
    return true;
}

std::set<size_t> SemanticLayer::getObjectsByName(const std::string &name)
{
    return objectMap[name];
}

std::set<size_t> SemanticLayer::getObjectsByNameInRange(const std::string &name, const point &minCorner, const point &maxCorner)
{
    auto nameSet = getObjectsByName(name);
    auto rangeSet = getObjectsInRange(minCorner, maxCorner);
    std::set<size_t> intersection;
    std::set_intersection(nameSet.begin(), nameSet.end(), rangeSet.begin(), rangeSet.end(), std::inserter(intersection, intersection.end()));
    return intersection;
}

std::set<size_t> SemanticLayer::getObjectsByNameInRange(const std::string &name, const polygon &pg)
{
    auto nameSet = getObjectsByName(name);
    auto rangeSet = getObjectsInRange(pg);
    std::set<size_t> intersection;
    std::set_intersection(nameSet.begin(), nameSet.end(), rangeSet.begin(), rangeSet.end(), std::inserter(intersection, intersection.end()));
    return intersection;
}

void SemanticLayer::addExampleObject()
{
    polygon testPoly;
    bg::append(testPoly.outer(), point(0.0, 0.0));
    bg::append(testPoly.outer(), point(0.0, 1.0));
    bg::append(testPoly.outer(), point(1.0, 0.0));
    bg::correct(testPoly);
    box bb = bg::return_envelope<box>(testPoly);
    point center;
    bg::centroid(testPoly, center);
    SemanticObject obj = {"TestTriangle", testPoly, center, bb};
    /*objectList.push_back(obj);
    objectRtree.insert(std::make_pair(bb, objectList.size() - 1));
    objectMap[obj.name].insert(objectList.size() - 1);*/
    addObject(obj);
}

void SemanticLayer::publishData()
{
    mapMsg.header.frame_id = tfFrame;
    mapMsg.header.stamp = ros::Time::now();
    semmapPub.publish(mapMsg);
}

void SemanticLayer::loadMapData(const std::string &file_name)
{
    if (parent == 0)
        return;

    this->file_name = file_name;
    clear();

    /*std::string map_file = parent->getLayerFile(file_name);
    std::istringstream istream(map_file);
    readMapData(istream);*/

    parent->getLayerFile(file_name, std::bind(&SemanticLayer::readMapData, this, std::placeholders::_1));

    publishData();
}

void SemanticLayer::saveMapData()
{
    if (parent == 0)
        return;

    /*std::ostringstream out;
    writeMapData(out);
    parent->putLayerFile(file_name, out.str());*/

    parent->putLayerFile(file_name, std::bind(&SemanticLayer::writeMapData, this, std::placeholders::_1));
}

bool SemanticLayer::readMapData(std::istream &input)
{
    YAML::Node map = YAML::Load(input);
    for (const YAML::Node &entry : map)
    {
        SemanticObject obj;
        obj.name = entry["name"].as<std::string>();
        /*for (const YAML::Node &p : entry["shape"])
        {
            bg::append(obj.shape.outer(), point(p[0].as<double>(), p[1].as<double>()));
            std::cout << "Added node: [" << p[0].as<double>() << ", " << p[1].as<double>() << "]" << std::endl;
        }*/
        try {
            bg::read_wkt(entry["shape"].as<std::string>(), obj.shape);
        } catch (const bg::read_wkt_exception &e) {
            ROS_ERROR_STREAM("Error reading object shape: " << e.what());
            return false;
        }
        bg::correct(obj.shape);
        obj.bounding_box = bg::return_envelope<box>(obj.shape);
        bg::centroid(obj.shape, obj.position);
        for (const YAML::Node &t : entry["tags"])
        {
            obj.tags.push_back(t.as<std::string>());
        }
        for (const YAML::Node &c : entry["confidence"])
        {
            obj.confidence.push_back(c.as<double>());
        }
        addObject(obj);
        std::cout << entry["name"] << std::endl;
    }
    return true;
}

bool SemanticLayer::writeMapData(std::ostream &output)
{
    YAML::Node map;
    for(const auto &map_entry : objectList)
    {
        const SemanticObject &obj = map_entry.second;
        YAML::Node n;
        n["name"] = obj.name;
        /*for (const point &p : obj.shape.outer())
        {
            YAML::Node yp;
            yp.push_back(p.x());
            yp.push_back(p.y());
            yp.SetStyle(YAML::EmitterStyle::Flow);
            n["shape"].push_back(yp);
        }
        n["shape"].SetStyle(YAML::EmitterStyle::Flow);*/
        std::ostringstream sh;
        sh << bg::wkt(obj.shape);
        n["shape"] = sh.str();
        for (const std::string &t : obj.tags)
        {
            n["tags"].push_back(t);
        }
        n["tags"].SetStyle(YAML::EmitterStyle::Flow);
        for (const double &c : obj.confidence)
        {
            n["confidence"].push_back(c);
        }
        n["confidence"].SetStyle(YAML::EmitterStyle::Flow);
        map.push_back(n);
    }
    output << map;
    return true;
}

/*std::string SemanticLayer::generateMapData()
{
    YAML::Node map;
    for(const auto &map_entry : objectList)
    {
        const SemanticObject &obj = map_entry.second;
        YAML::Node n;
        n["name"] = obj.name;
        for (const point &p : obj.shape.outer())
        {
            YAML::Node yp;
            yp.push_back(p.x());
            yp.push_back(p.y());
            yp.SetStyle(YAML::EmitterStyle::Flow);
            n["shape"].push_back(yp);
        }
        n["shape"].SetStyle(YAML::EmitterStyle::Flow);
        map.push_back(n);
    }
    std::stringstream stream;
    stream << map;
    return stream.str();
}*/

void SemanticLayer::printQuery()
{
    box query_box(point(0.5, 0.51), point(5, 5));
    polygon testPoly;
    bg::append(testPoly.outer(), point(0.0, 0.0));
    bg::append(testPoly.outer(), point(0.0, 1.0));
    bg::append(testPoly.outer(), point(1.0, 0.0));
    bg::correct(testPoly);
    std::vector<rtree_entry> result_s;
    objectRtree.query(bgi::intersects(query_box), std::back_inserter(result_s));
    ROS_INFO("Printing found objects");
    for (rtree_entry val : result_s)
    {
        SemanticObject foundObject = objectList[val.second];
        bool intersects = bg::intersects(query_box, foundObject.shape);
        ROS_INFO("Found Object: %s, Intersects: %d\n", foundObject.name.c_str(), intersects);
    }
}

}
