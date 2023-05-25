#include "main.h"



// SECTION: DEVICE CONFIGURATION, keep global so it can be used throuhgout the program


// Okapilib Controller
Controller controller;



// okapilib Chassis

std::shared_ptr<ChassisController> myChassis =
  ChassisControllerBuilder()
    .withMotors({10, -9}, {-1, 2})
    // Green gearset, 4 in wheel diam, 11.5 in wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4.25_in, 11.5_in}, imev5GreenTPR})
    // TODO: SET UP AND TUNE PID??????
    .withMaxVelocity(100)
    .build();

std::shared_ptr<AsyncMotionProfileController> profileController = 
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(myChassis)
    .buildMotionProfileController();

// end OKAPILIB Chassis

// catapult motor
Motor catapult(11, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

// catapult touch sensor (to figure out of it is down)
auto catapultLimitSwitch = ADIButton('A', false);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  
	myChassis->getModel()->setBrakeMode(AbstractMotor::brakeMode::coast);
  // tank drive
  
  pros::lcd::initialize();
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
void competition_initialize() {}

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
	profileController->generatePath({
    {106_in, 14_in, 0_deg},
    {128_in, 13_in, 0_deg}},
    "goToMatchLoadZone1");
	profileController->setTarget("goToMatchLoadZone1");
	profileController->waitUntilSettled();
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
void opcontrol() {
  myChassis->getModel()->setBrakeMode(AbstractMotor::brakeMode::coast);
  ControllerButton runCat(ControllerDigital::X);
  catapult.setBrakeMode(AbstractMotor::brakeMode::coast);
  // tank drive
  
  while (true) {
    myChassis->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY),
                                  controller.getAnalog(ControllerAnalog::rightY));
            // run the catapult when X is pressed
        if (runCat.isPressed()) {
            catapult.moveAbsolute(-150, 100);
            pros::delay(100);
        }
        else {
            catapult.moveAbsolute(0, 100);
            pros::delay(100);
        }

    pros::delay(20);
    
  }

}

// SECTION: FUNCTIONS

/*
 * @brief: catapult control function
 * @param: number of times to launch
*/
void launch(int numLaunches = 1) {
  for (int i = 0; i < numLaunches; i++) {
    while(!catapultLimitSwitch.isPressed()) {
      catapult.moveVelocity(100);
      pros::delay(20);
    }
    catapult.moveVelocity(0);
  }
}