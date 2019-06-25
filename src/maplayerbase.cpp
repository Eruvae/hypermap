#include "maplayerbase.h"

#include "hypermap.h"

namespace hypermap
{

void MapLayerBase::test()
{
    parent->testZip();
}

void MapLayerBase::setSubscribeMode(bool mode)
{
    subscribe_mode = mode;
}

}
