#pragma once

#include "pros/rtos.hpp"
#include <print>
#define log(...)                                                               \
  std::println("[{}]\t{}\t({}() @ {}:{})", pros::micros(),                     \
               std::format(__VA_ARGS__), __FUNCTION__, __FILE__, __LINE__)
