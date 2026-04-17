#include "auton/quadrant.h"

namespace auton {
Quadrant Quadrant::fromSide(COLOR color, SIDE side) {
  SIGN sign;
  switch (color) {
  case COLOR::RED:
    switch (side) {
    case SIDE::LEFT:
      sign = SIGN::REFEREE;
      break;
    case SIDE::RIGHT:
      sign = SIGN::AUDIENCE;
      break;
    }
    break;
  case COLOR::BLUE:
    switch (side) {
    case SIDE::RIGHT:
      sign = SIGN::REFEREE;
      break;
    case SIDE::LEFT:
      sign = SIGN::AUDIENCE;
      break;
    }
    break;
  }
  return {color, sign};
}

Quadrant Quadrant::RED_LEFT = fromSide(COLOR::RED, SIDE::LEFT);
Quadrant Quadrant::RED_RIGHT = fromSide(COLOR::RED, SIDE::RIGHT);
Quadrant Quadrant::BLUE_LEFT = fromSide(COLOR::BLUE, SIDE::LEFT);
Quadrant Quadrant::BLUE_RIGHT = fromSide(COLOR::BLUE, SIDE::RIGHT);

Quadrant::SIDE Quadrant::side() const {
  if (color == COLOR::RED)
    return sign == SIGN::REFEREE ? SIDE::LEFT : SIDE::RIGHT;
  else
    return sign == SIGN::REFEREE ? SIDE::RIGHT : SIDE::LEFT;
}

bool Quadrant::isLeft() const { return side() == SIDE::LEFT; }

bool Quadrant::isRight() const { return side() == SIDE::RIGHT; }

void Quadrant::setSide(SIDE side) { sign = fromSide(color, side).sign; }

} // namespace auton