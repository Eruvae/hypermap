#include "maplayerbase.h"

#include "hypermap.h"

using namespace hypermap;

void MapLayerBase::test()
{
    parent->testZip();
}

void MapLayerBase::setSubscribeMode(bool mode)
{
    subscribe_mode = mode;
}

void MapLayerBase::getRGBA()
{

}
