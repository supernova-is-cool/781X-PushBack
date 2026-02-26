#include "main.h"
#include "auton/autons.h"
#include "autonSelector/selector.hpp"
#include "lemlib/api.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "robot.h"

ts::selector *selector = nullptr;
ts::auton rightRush("Right Rush", autons::rightRush);
ts::auton leftRush("Left Rush", autons::leftRush);
// ts::auton right9Ball("Right 9 Ball", autons::right9Ball);
ts::auton leftMiddle("Left Middle", autons::leftMiddle);
// ts::auton skillsawp("Skill SAWP", autons::skillsAWP);
// ts::auton skills("[[BAD!!!]] Skills", autons::skills);
// ts::auton skillsPark("Skills Park", autons::skillsPark);
// ts::auton soloWinPoint("SAWP", autons::soloWinPoint);
// ts::auton center("Center????", autons::center);
ts::auton doNothing("Do Nothing", autons::doNothing);
ts::auton skills("Skills", autons::skills);

void screen() {
  struct LabeledMotor {
    std::string name;
    pros::MotorGroup &motor;

    float mean_temp() const {
      auto temps = motor.get_temperature_all();
      float sum = 0;
      for (auto temp : temps) {
        sum += temp;
      }
      return sum / temps.size();
    }
  };
  std::vector<int8_t> intake_ports =
      bot.m_config.motors.bottomStage.get_port_all();
  {
    std::vector<int8_t> top_intake_ports =
        bot.m_config.motors.topStage.get_port_all();
    intake_ports.insert(intake_ports.end(), top_intake_ports.begin(),
                        top_intake_ports.end());
  }
  pros::MotorGroup intake{intake_ports};
  std::vector<LabeledMotor> motors{
      LabeledMotor{"Left", bot.m_config.motors.left},
      LabeledMotor{"Right", bot.m_config.motors.right},
      LabeledMotor{"Intake", intake},
  };
  while (true) {
    if (pros::lcd::is_initialized()) {
      pros::lcd::print(0, "Auton:\t%s",
                       selector->get_selected_auton_name().c_str());
      pros::lcd::print(3, "x:\t%fin", bot.getPose().x);
      pros::lcd::print(4, "y:\t%fin", bot.getPose().y);
      pros::lcd::print(5, "theta:\t%fdeg", bot.getPose().theta);

      std::string labels = "";
      std::string temps = "";
      for (const auto &motor : motors) {
        labels += motor.name;
        temps += std::format("{:.1f}C", motor.mean_temp());
        // Even out the lengths
        size_t max_len = std::max(labels.size(), temps.size()) + 2;
        labels += std::string(max_len - labels.size(), ' ');
        temps += std::string(max_len - temps.size(), ' ');
      }
      pros::lcd::print(6, "%s", labels.c_str());
      pros::lcd::print(7, "%s", temps.c_str());
    }
    pros::delay(50);
  }
}

void init_odom_printing() {
  if (!pros::lcd::is_initialized()) {
    selector->hide();
    pros::lcd::initialize();
    new pros::Task{screen};
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  selector = ts::selector::get();
  // ensure robot is initialized
  Robot::get();

  if (pros::competition::is_connected() && pros::competition::is_disabled()) {
    selector->display();
  } else {
    init_odom_printing();
  }

  pros::delay(250);
  bot.calibrate();
  pros::delay(250);

  bot.setAlliance(ALLIANCE::RED);
  bot.setPose({0, 0, 0}, 72);
  pros::delay(250);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  if (!selector->is_auton_selected()) {
    // Handle no selected auton
    // TODO: Change to doNothing before comp
    selector->select_auton(skills.name);
  }
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  init_odom_printing();
  if (!selector->is_auton_selected()) {
    // Handle no selected auton
    // TODO: Change to doNothing before comp
    selector->select_auton(skills.name);
  }
  //bot.moveToPoint({0, 24, 0}, 2000);
  //bot.turnToHeading(90, 2000);
  // autons::skills();
  //bot.odomLift.extend();
  selector->run_selected_auton();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

/*void opcontrol() {
        pros::Controller master(pros::E_CONTROLLER_MASTER);
        pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with
forwards ports 1 & 3 and reversed port 2 pros::MotorGroup right_mg({-4, 5, -6});
// Creates a motor group with forwards port 5 and reversed ports 4 & 6


        while (true) {
                pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() &
LCD_BTN_LEFT) >> 2, (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
                                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >>
0);  // Prints status of the emulated screen LCDs

                // Arcade control scheme
                int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount
forward/backward from left joystick int turn =
master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right
joystick left_mg.move(dir - turn);                      // Sets left motor
voltage right_mg.move(dir + turn);                     // Sets right motor
voltage pros::delay(20);                               // Run for 20 ms then
update
        }
} */