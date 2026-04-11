#include "config.h"

robotConfig::Tunables robotConfig::Tunables::tunables{
    .lateralController =
        lemlib::ControllerSettings{
            15,  // kP
            0,   // kI
            55,  // kD
            3,   // windup range
            1,   // smallErrorRange
            100, // smallErrorTimeout
            3,   // largeErrorRange
            500, // largeErrorTimeout
            0    // slew rate
        },
    .angularController =
        lemlib::ControllerSettings{
            4,   // kP
            0,   // kI
            35,  // kD
            15,  // windup range
            1,   // smallErrorRange
            100, // smallErrorTimeout
            3.5, // largeErrorRange
            350, // largeErrorTimeout
            0    // slew rate
        },
    .horizontalDrift = 10.0,
    .imuGain = 1800. / 1781.23,
    .driveCurve = lemlib::ExpoDriveCurve{0, 0, 1}};