#include "lemlib/chassis/chassis.hpp"

class CustomIMU : public pros::IMU {
  public:
    CustomIMU(int port, double scalar)
      : pros::IMU(port),
        m_port(port),
        m_scalar(scalar) {}
    virtual double get_rotation() const override {
      return pros::c::imu_get_rotation(m_port) * m_scalar;
    }
  private:
    const int m_port;
    const double m_scalar;
};

// create custom IMU with a scalar of 1.1
//CustomIMU my_imu(14, 1.1);

