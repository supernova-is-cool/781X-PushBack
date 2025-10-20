#include "lemlib/chassis/chassis.hpp"
#include "config.h"

robotConfig::Tunables robotConfig::Tunables::tunables {
    .lateralController =
        lemlib::ControllerSettings {
            15.5, // kP
            0, // kI
            95, // kD
            3, // windup range
            1, // smallErrorRange
            100, // smallErrorTimeout
            3, // largeErrorRange
            500, // largeErrorTimeout
            0 // slew rate
        },
    .angularController =
        lemlib::ControllerSettings {
            6.25, // kP
            0, // kI
            58.5, // kD
            15, // windup range
            1, // smallErrorRange
            100, // smallErrorTimeout
            3.5, // largeErrorRange
            350, // largeErrorTimeout
            0 // slew rate
        },
    .horizontalDrift = 10.0,
    .imuGain = 0.0,
    .driveCurve = lemlib::ExpoDriveCurve {0, 0, 1}};