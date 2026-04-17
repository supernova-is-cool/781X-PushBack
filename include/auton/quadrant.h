#pragma once

#include "color.h"

namespace auton {
/** Describes a quadrant of the field. */
struct Quadrant {
  /** Left/Right from perspective of the nearest alliance station. */
  enum class SIDE { LEFT, RIGHT };
  /**
   * @brief Refers to the sign of the corners on that side of the field.
   *
   * @note The name "SIGN" is a remnant from High Stakes
   */
  enum class SIGN {
    /** @deprecated use Quadrant::SIGN::REFEREE */
    POSITIVE,
    /** @deprecated use Quadrant::SIGN::AUDIENCE */
    NEGATIVE,
    REFEREE = POSITIVE,
    AUDIENCE = NEGATIVE,
  };
  /** Nearest alliance station. */
  COLOR color;
  /** Use referee and audience enum members. */
  SIGN sign;

  static Quadrant fromSide(COLOR color, SIDE side);

  static Quadrant RED_LEFT;
  static Quadrant RED_RIGHT;
  static Quadrant BLUE_LEFT;
  static Quadrant BLUE_RIGHT;

  static constexpr SIDE oppositeSide(SIDE side) {
    return (side == SIDE::LEFT) ? SIDE::RIGHT : SIDE::LEFT;
  }

  SIDE side() const;
  bool isLeft() const;
  bool isRight() const;

  void setSide(SIDE side);
};
} // namespace auton