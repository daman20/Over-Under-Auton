#include "main.h"



// SECTION: DEVICE CONFIGURATION


// Okapilib Controller
Controller controller;



// okapilib Chassis

std::shared_ptr<ChassisController> myChassis =
  ChassisControllerBuilder()
    .withMotors({10, -9}, {-1, 2})
    // blue gearset, 4 in wheel diam, 11.5 in wheel track
    .withDimensions(AbstractMotor::gearset::blue, {{4.25_in, 11.5_in}, imev5BlueTPR})
    // TODO: SET UP AND TUNE PID??????
    .withMaxVelocity(100)
    .withOdometry()
    .buildOdometry();

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
Motor catapult(11, true, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);

// catapult touch sensor (to figure out of it is down)
auto catapultLimitSwitch = ADIButton('A', false);

// intake motor
Motor intake(3, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

// acorn touch sensor to detect whether or not an acorn is loaded
auto acornTouch = OpticalSensor(5, OpticalSensorOutput::hue, true);

// END SECTION: DEVICE CONFIGURATION

// START SECTION: HELPER FUNCTIONS

/*
 * @brief: catapult control function
 * @param: number of times to launch
*/
void launch(int numLaunches = 1) {
  for (int i = 0; i < numLaunches; i++) { //repeat for the number of times to launch
    catapult.moveRelative(50, 100); // move the catapult up 50 degrees so that the acorn is launched
    // RELOAD: move the catapult down until the limit switch is pressed
    while(!catapultLimitSwitch.isPressed()) { // check if the limit switch is not pressed, meaning the catapult isn't down
      catapult.moveVelocity(100); // move the catapult down
      pros::delay(20); //delay to not overload
    }
    catapult.moveVelocity(0); // stop the catapult when done
  }
}

// END SECTION: HELPER FUNCTIONS


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
  // launch acorns as they are loaded
  while(true){
    if (acornTouch.getHue() < 100 && acornTouch.getHue() > 80) {
      launch();
    }
  }
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
  myChassis->getModel()->setBrakeMode(AbstractMotor::brakeMode::brake);
  ControllerButton runCat(ControllerDigital::X);
  catapult.setBrakeMode(AbstractMotor::brakeMode::coast);
  // tank drive
  while (true) {
    myChassis->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY),
                                  controller.getAnalog(ControllerAnalog::rightY));
            // run the catapult when X is pressed
        if (runCat.isPressed()) {
            catapult.moveVelocity(20);
            pros::delay(100);
        }
        else {
            catapult.moveVoltage(0);
            pros::delay(100);
        }

    pros::delay(20);
    
  }

}

