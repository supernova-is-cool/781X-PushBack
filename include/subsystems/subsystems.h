#pragma once

#include "lemlib/api.hpp"
#include "lemlib/timer.hpp"
#include <memory>

class subsystem {

private:
  std::unique_ptr<pros::Task> task;
  void taskRunner() {
    while (true) {
      runTask();
      pros::delay(10);
    }
  };

protected:
  virtual void runTask() = 0;
  lemlib::Timer timer = {0};

public:
  subsystem() {
    task = std::make_unique<pros::Task>([this] { taskRunner(); });
    task->resume();
  }
  virtual ~subsystem() {
    if (task) {
      task->remove();
    }
  };
};