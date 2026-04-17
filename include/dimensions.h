#pragma once

namespace dimensions {
namespace field {
namespace long_headings {
/** Heading to face referee. This side of the field is furthest from the climb
 * stake. */
constexpr float REFEREE = 0;
/** Heading to face audience. This side of the field is closest to the climb
 * stake. */
constexpr float AUDIENCE = 180;
/** Heading to face red station */
constexpr float RED_STATION = 270;
/** Heading to face blue station */
constexpr float BLUE_STATION = 90;
}; // namespace long_headings

using namespace long_headings;

namespace short_headings {
/** Heading to face referee. This side of the field is furthest from the climb
 * stake. */
constexpr float REF = long_headings::REFEREE;
/** Heading to face referee (stands). This side of the field is furthest from
 * the climb stake. */
constexpr float STANDS = long_headings::AUDIENCE;
/** Heading to face red driver station */
constexpr float RED = long_headings::RED_STATION;
/** Heading to face blue driver station */
constexpr float BLUE = long_headings::BLUE_STATION;

/** Distance between wall and barrier parallel to y axis */
const float PARK_ZONE_DEPTH = 17 /* in */;

/** Distance between parallel park barriers */
const float PARK_ZONE_WIDTH = 18 /* in */;

} // namespace short_headings
namespace DIR = short_headings;

constexpr float FIELD_LENGTH = 140.25;

constexpr float TILE = FIELD_LENGTH / 6;

/** x axis goes from red driver station to the blue driver station */
constexpr float MAX_X = TILE * 3;
constexpr float MIN_X = -MAX_X;
/** x axis goes from red driver station to the blue driver station */
constexpr float MAX_Y = TILE * 3;
constexpr float MIN_Y = -MAX_Y;

/** Diameter of the rings */
constexpr float BALL_INNER_DIAM = 3.25;
constexpr float CENTER_GOAL_RADIUS = 22.5 / 2.;
} // namespace field

namespace robot {
constexpr float TRACK_WIDTH = 11.5;
constexpr float DRIVE_WIDTH = 27.0 / 2;
/** Includes funnels */
constexpr float DRIVE_LENGTH = 31.0 / 2;
/** Distance from center of robot to axle of match loader */
constexpr float MATCHLOADER_DIST_TO_CENTER = 11.;

} // namespace robot

namespace all {
using namespace field;
using namespace robot;
} // namespace all
} // namespace dimensions