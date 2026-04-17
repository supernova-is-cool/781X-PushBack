#include "auton/transform.h"
#include "robot.h"

namespace auton {
TransformLockGuard::TransformLockGuard(
    std::shared_ptr<Transformation> newTransform) {
  // Store the old transform and set the new one
  m_oldTransform = bot.m_transform;
  bot.setTransform(ChainedTransform::make(newTransform, m_oldTransform));
}
TransformLockGuard::~TransformLockGuard() {
  // Reset the old transform
  bot.setTransform(m_oldTransform);
}
} // namespace auton