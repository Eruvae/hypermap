#ifndef HYPERMAPDISPLAY_H
#define HYPERMAPDISPLAY_H

#include "rviz/display.h"
#include "rviz/display_group.h"
#include "rviz/default_plugin/image_display.h"

namespace hypermap {

class HypermapDisplay : public rviz::DisplayGroup
{
Q_OBJECT
public:
  HypermapDisplay();

  // Overrides from Display
  //virtual void onInitialize();
  //virtual void fixedFrameChanged();
  //virtual void reset();

protected Q_SLOTS:
  void updateLayerProps();
  //void updateMap();
  //* If this is true, will disable it's children when it's own bool value is false */

protected:
  rviz::IntProperty *layerCnt_;
  rviz::BoolProperty **enableLayer_;
  rviz::BoolProperty *enable_bg_property_;

private:
  int oldLayerCnt;
};

}
#endif // HYPERMAPDISPLAY_H
