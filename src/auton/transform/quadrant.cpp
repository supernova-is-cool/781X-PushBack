#include "auton/transform.h"

namespace auton {
auton::QuadrantTransform::QuadrantTransform(Quadrant quadrant,
                                            Quadrant normalQuadrant)
    : ChainedTransform(
          std::make_shared<AllianceTransform>(quadrant.color,
                                              normalQuadrant.color),
          std::make_shared<SignTransform>(quadrant.sign, normalQuadrant.sign)) {
}
} // namespace auton