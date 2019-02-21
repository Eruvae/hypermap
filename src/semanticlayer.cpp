#include "semanticlayer.h"

#include <yaml-cpp/yaml.h>

#include "hypermap.h"

using namespace hypermap;

/*SemanticLayer::SemanticLayer() : MapLayerBase("map")
{

}*/

void SemanticLayer::polygonToTriangles(const ring &pg)
{
    //boost::polygon::voronoi_diagram<double> vd;
    //boost::polygon::construct_voronoi(pg.begin(), pg.end(), &vd);

}

void SemanticLayer::updateMap(const hypermap_msgs::SemanticMapUpdate::ConstPtr update)
{
    for (const hypermap_msgs::SemanticObject &obj : update->to_add)
    {
        addObject(createSemanicObjFromMessage(obj));
    }
    for (size_t id : update->to_modify)
    {
        updateObject(id, createSemanicObjFromMessage(update->updates[id]));
    }
    for (size_t id : update->to_delete)
    {
        removeObject(id);
    }
}

void SemanticLayer::addObject(const SemanticObject &newObject)
{
    objectList[next_index] = newObject;
    objectMap[newObject.name].insert(next_index);
    objectRtree.insert(std::make_pair(newObject.bounding_box, next_index));
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
    }
    return true;
}

bool SemanticLayer::removeObject(size_t id)
{
    SemanticObject &object = objectList[id];
    objectMap[object.name].erase(id);
    objectRtree.remove(std::make_pair(object.bounding_box, id));
    objectList.erase(id);
    return true;
}

hypermap_msgs::SemanticObject SemanticLayer::semanticObjectToMsg(const SemanticObject &obj)
{
    hypermap_msgs::SemanticObject msg;
    msg.name = obj.name;
    msg.shape = boostToPolygonMsg(obj.shape);
    return msg;
}

SemanticLayer::SemanticObject SemanticLayer::createSemanicObjFromMessage(const hypermap_msgs::SemanticObject &msg)
{
    SemanticObject obj;
    obj.name = msg.name;
    obj.shape = polygonMsgToBoost(msg.shape);
    obj.bounding_box = bg::return_envelope<box>(obj.shape);
    return obj;
}

geometry_msgs::Point SemanticLayer::boostToPointMsg(const point &p)
{
    geometry_msgs::Point pm;
    pm.x = p.x(); //p.get<0>();
    pm.y = p.y(); //p.get<1>();
    return pm;
}

geometry_msgs::Point32 SemanticLayer::boostToPoint32Msg(const point &p)
{
    geometry_msgs::Point32 pm;
    pm.x = p.x(); //get<0>();
    pm.y = p.y(); //get<1>();
    return pm;
}

geometry_msgs::Polygon SemanticLayer::boostToPolygonMsg(const polygon &pg)
{
    geometry_msgs::Polygon pgm;
    for (const auto &p : pg.outer())
        pgm.points.push_back(boostToPoint32Msg(p));
    return pgm;
}

SemanticLayer::point SemanticLayer::pointMsgToBoost(const geometry_msgs::Point &pm)
{
    return point(pm.x, pm.y);
}

SemanticLayer::point SemanticLayer::point32MsgToBoost(const geometry_msgs::Point32 &pm)
{
    return point(pm.x, pm.y);
}

SemanticLayer::polygon SemanticLayer::polygonMsgToBoost(const geometry_msgs::Polygon &pgm)
{
    polygon pg;
    for (const auto &p : pgm.points)
      bg::append(pg.outer(), point32MsgToBoost(p));
    return pg;
}

int SemanticLayer::getIntValue(double xPos, double yPos)
{
    return 0;
}

std::string SemanticLayer::getStringValue(double xPos, double yPos)
{
    return "";
}

std::set<size_t> SemanticLayer::getObjectsAt(const point &p)
{
    std::vector<rtree_entry> result_obj;
    std::set<size_t> result;
    objectRtree.query(bgi::covers(p), std::back_inserter(result_obj));
    for (rtree_entry val : result_obj)
    {
        const SemanticObject &foundObject = objectList[val.second];
        if (bg::covered_by(p, foundObject.shape))
            result.insert(val.second);
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

std::vector<std::pair<geometry_msgs::Point, std::string>> SemanticLayer::getStringReps(const geometry_msgs::Polygon &area)
{
    std::set<size_t> qres = getObjectsInRange(polygonMsgToBoost(area));
    std::vector<std::pair<geometry_msgs::Point, std::string>> res;
    for (size_t i : qres)
    {
        const SemanticObject &obj = objectList[i];
        point center;
        bg::centroid(obj.shape, center);
        res.push_back(std::make_pair(boostToPointMsg(center), obj.name));
    }
    return res;
}

void SemanticLayer::addExampleObject()
{
    polygon testPoly;
    bg::append(testPoly.outer(), point(0.0, 0.0));
    bg::append(testPoly.outer(), point(0.0, 1.0));
    bg::append(testPoly.outer(), point(1.0, 0.0));
    box bb = bg::return_envelope<box>(testPoly);
    SemanticObject obj = {"TestTriangle", testPoly, bb};
    /*objectList.push_back(obj);
    objectRtree.insert(std::make_pair(bb, objectList.size() - 1));
    objectMap[obj.name].insert(objectList.size() - 1);*/
    addObject(obj);
}

void SemanticLayer::loadMapData(const std::string &file_name)
{
    this->file_name = file_name;
    //std::string map_file = parent->getLayerFile(file_name);
    //std::istringstream istream(map_file);
    //readMapData(istream);
    parent->getLayerFile(file_name, std::bind(&SemanticLayer::readMapData, this, std::placeholders::_1));
}

void SemanticLayer::saveMapData()
{
    //std::ostringstream out;
    //writeMapData(out);
    //parent->putLayerFile(file_name, out.str());
    parent->putLayerFile(file_name, std::bind(&SemanticLayer::writeMapData, this, std::placeholders::_1));
}

void SemanticLayer::readMapData(std::istream &input)
{
    YAML::Node map = YAML::Load(input);
    for (const YAML::Node &entry : map)
    {
        polygon shape;
        for (const YAML::Node &p : entry["shape"])
        {
            bg::append(shape.outer(), point(p[0].as<double>(), p[1].as<double>()));
            std::cout << "Added node: [" << p[0].as<double>() << ", " << p[1].as<double>() << "]" << std::endl;
        }
        SemanticObject obj = {entry["name"].as<std::string>(), shape, bg::return_envelope<box>(shape)};
        addObject(obj);
        std::cout << entry["name"] << std::endl;
    }
}

void SemanticLayer::writeMapData(std::ostream &output)
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
    output << map;
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
