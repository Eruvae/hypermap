#ifndef HYPERMAPDISPLAY_H
#define HYPERMAPDISPLAY_H

#include "rviz/display.h"
#include "rviz/image/image_display_base.h"

class HypermapDisplay : public rviz::Display
{
Q_OBJECT
public:
  HypermapDisplay();

  // Overrides from Display
  //virtual void onInitialize();
  //virtual void fixedFrameChanged();
  //virtual void reset();

protected Q_SLOTS:
  //void updateMap();

protected:
  BoolProperty* enable_bg_property_;
};

#endif // HYPERMAPDISPLAY_H
