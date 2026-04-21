#include "auton/autons.h"
#include "dimensions.h"
#include "robot.h"
void autons::doNothing() {
  bot.setPose({0, 0, dimensions::all::RED_STATION});
  bot.moveToX(-4, 1000);
}