#include "config.h"

robotConfig::Tunables robotConfig::Tunables::tunables{
    .lateralController =
        lemlib::ControllerSettings{
            10,  // kP
            0,   // kI
            70,  // kD
            3,   // windup range
            1,   // smallErrorRange
            100, // smallErrorTimeout
            3,   // largeErrorRange
            500, // largeErrorTimeout
            0    // slew rate
        },
    .angularController =
        lemlib::ControllerSettings{
            2.75,   // kP
            0,   // kI
            26.75,  // kD
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