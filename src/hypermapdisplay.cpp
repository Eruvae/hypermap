#include "hypermapdisplay.h"

HypermapDisplay::HypermapDisplay() : rviz::Display()
{
    enable_bg_property_ = new BoolProperty("Enable background", true, "Display background of map", this);
}

/*void HypermapDisplay::onInitialize()
{
    Display::onInitialize();
}

void HypermapDisplay::fixedFrameChanged()
{
    Display::fixedFrameChanged();
}

void HypermapDisplay::reset()
{
    Display::reset();
}

void HypermapDisplay::updateMap()
{

}*/
