#include "maplayerbase.h"
#include "hypermap.h"

void MapLayerBase::test()
{
    parent->testZip();
}

void MapLayerBase::setSubscribeMode(bool mode)
{
    subscribe_mode = mode;
}
