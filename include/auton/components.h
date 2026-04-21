#pragma once

#include "auton/quadrant.h"
#include <cstddef>

namespace auton::components {
/**
 * @brief Intakes blocks from the matchloader.
 *
 * @param quadrant Specifies the matchloader to intake from.
 * @param onlyMyColor If set to true, intakes only 3 blocks from matchloader,
 * otherwise intakes all 6 blocks.
 */
void matchload(Quadrant quadrant, bool onlyMyColor);
/**
 * @brief Intakes the blocks from the center.
 *
 * @param quadrant
 * @param alignWithGoal Whether to align the bot with the center goal. Helps
 * when scoring on the center goal afterwards.
 */
void grabCenterBlocks(Quadrant quadrant, bool alignWithGoal = true);
/**
 * @brief Scores blocks into the long goal.
 * @pre Starts in a position where the bot can back into the long goal easily
 * (ideally matchloader).
 *
 * @param quadrant
 */
void scoreLong(Quadrant quadrant);
/**
 * @brief Scores blocks into the center goal.
 * @pre The robot center must be aligned with the line of the center goal.
 *
 * @param quadrant
 * @param score_duration Specifies how long to spend scoring. Enables imprecise
 * control of number of blocks scored.
 */
void scoreCenter(Quadrant quadrant, std::size_t score_duration);

enum class LongDescoreSide { OUTER, INNER };

/**
 * @brief Uses descore to push blocks in the long goal into the control zone.
 * @pre Starts from long goal scoring position.
 *
 * @param quadrant
 * @param side
 */
void pushLong(Quadrant quadrant, LongDescoreSide side);

} // namespace auton::components