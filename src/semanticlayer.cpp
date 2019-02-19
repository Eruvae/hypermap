#include "semanticlayer.h"
#include "hypermap.h"
#include "yaml-cpp/yaml.h"

/*SemanticLayer::SemanticLayer() : MapLayerBase("map")
{

}*/

int SemanticLayer::getIntValue(double xPos, double yPos)
{
    return 0;
}

std::string SemanticLayer::getStringValue(double xPos, double yPos)
{
    return "";
}

void SemanticLayer::loadMapData(const std::string &file_name)
{

}

void SemanticLayer::readMapData(const std::string &data)
{
    YAML::Node map = YAML::Load(data);
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

std::string SemanticLayer::generateMapData()
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
}
