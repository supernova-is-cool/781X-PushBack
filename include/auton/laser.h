#pragma once

#include "math/NlPose.h"
#include "pros/distance.hpp"

class Laser {
public:
  /** @returns The distance measured by the laser in inches. */
  std::optional<float> measure();

  /**
   * @brief Calculates the bot's x coordinate from the laser measurement,
   * assuming the laser hits a vertical wall.
   * @param botTheta The current heading of the bot in radians. Must be in
   * standard radians (0deg -> +x, 90deg -> +y).
   */
  std::optional<float> getBotX(float botTheta);
  /**
   * @brief Calculates the bot's y coordinate from the laser measurement,
   * assuming the laser hits a horizontal wall.
   */
  std::optional<float> getBotY(float botTheta);

  /**
   * @brief Attempts to reset the x-coordinate of the bot, using this laser.
   * Assumes the laser hits a vertical wall.
   * @return True if the reset was successful, false otherwise.
   */
  bool resetX();
  /**
   * @brief Attempts to reset the y-coordinate of the bot, using this laser.
   * Assumes the laser hits a horizontal wall.
   * @return True if the reset was successful, false otherwise.
   */
  bool resetY();

  Laser(size_t port, NlPose offset);

private:
  pros::Distance m_sensor;
  /** Offset from center of robot. Theta must be in std radians. X corresponds
   * to forwards-backwards and Y corresponds to left-right. */
  const NlPose m_offset;
};