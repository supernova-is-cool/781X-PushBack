#pragma once

#include "color.h"
#include <functional>
#include <optional>

namespace auton {
struct Auton {
    char* label;
    std::function<void(ALLIANCE)> run;
    std::optional<char*> labelForController;
    /** Displayed less prominently than label on brain */
    char* description;
};

namespace autons {

} // namespace autons
} // namespace auton