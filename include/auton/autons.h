#pragma once

#include "color.h"
#include <functional>
#include <optional>

namespace autons {
  void doNothing();
  void rightRush();
  /** Starts on left corner, scores on left middle goal, and left long goal. */
  void leftMiddle();
  void center();
  void skills();
  void soloWinPoint();
}