#include "main.h"
#include "autoSelect/selection.h"

// SECTION: CONSTANTS

auto startingState = OdomState{1000_mm, -1400_mm, 0_deg};
auto goalLocation = Point{1200_mm, 0_mm};
auto matchLoadZone1 = Point{-1400_mm, 1400_mm};
auto matchLoadZone2 = Point{-1400_mm, -1400_mm};

// SECTION: DEVICE CONFIGURATION


// Okapilib Controller
Controller controller;



// okapilib Chassis

std::shared_ptr<OdomChassisController> chassis =
  ChassisControllerBuilder()
    .withMotors({17, -16}, {-7, 6})
    // blue gearset, 4 in wheel diam, 11.5 in wheel track (center-to-center distance between the wheels (center-to-center meaning the width between the centers of both wheels))
    .withDimensions(AbstractMotor::gearset::blue, {{4.25_in, 11.5_in}, imev5BlueTPR})
    // TODO: SET UP AND TUNE PID??????
    .withMaxVelocity(100)
    .withOdometry()
    .buildOdometry();

// profile controller for autonomous to allow for preplanned routes
std::shared_ptr<AsyncMotionProfileController> profileController = 
  AsyncMotionProfileControllerBuilder()
    .withLimits({
      1.0, // Maximum linear velocity of the Chassis in m/s
      2.0, // Maximum linear acceleration of the Chassis in m/s/s
      10.0 // Maximum linear jerk of the Chassis in m/s/s/s
    })
    .withOutput(chassis)
    .buildMotionProfileController();

// end OKAPILIB Chassis

// catapult motors
Motor cat1(-5, true, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);
Motor cat2(15, false, AbstractMotor::gearset::red, AbstractMotor::encoderUnits::degrees);
MotorGroup catapult({cat1, cat2});

// catapult touch sensor (to figure out of it is down)
auto catapultLimitSwitch = ADIButton('A', false);

// intake motor
Motor intake(20, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

// acorn touch sensor to detect whether or not an acorn is loaded
auto acornLoad = OpticalSensor(5, OpticalSensorOutput::hue, true);



pros::ADIDigitalOut matchLoadArm ('H');


// END SECTION: DEVICE CONFIGURATION

// START SECTION: HELPER FUNCTIONS

/*
 * @brief: catapult control function
 * @param: number of times to launch
*/
void launch(int numLaunches = 1, bool aimbot = false) {
  if(aimbot){
    chassis->turnToPoint(goalLocation); // AIMBOT: turn to face the goal
  }
  catapult.setBrakeMode(AbstractMotor::brakeMode::brake); // set the catapult to brake to hold at the bottom
  for (int i = 0; i < numLaunches; i++) { //repeat for the number of times to launch
    catapult.moveVelocity(-100); // move the catapult down
    pros::delay(1000); //wait so that the catapult can launch without hitting touch sensor
    // RELOAD: move the catapult down until the limit switch is pressed
    while(!catapultLimitSwitch.changedToPressed()) { // check if the limit switch is not pressed, meaning the catapult isn't down
      pros::delay(40); //delay not to overload
    }
    pros::delay(5);
    catapult.moveVelocity(0); // stop the catapult when done
  }
}

void matchLoadAutoLaunch(){
  chassis->driveToPoint(matchLoadZone1); // drive to the match load zone
  // launch acorns as they are loaded
  while(true){
    if (acornLoad.getHue() < 100 && acornLoad.getHue() > 80) { // if it is green
      launch();
    }
    pros::delay(20); // delay to not overload
  }
}
// END SECTION: HELPER FUNCTIONS

// START SECTION AUTONOMOUS ROUTINES
void offensive() {
  // set drivetrain speed
  chassis->setMaxVelocity(400);
  //turn on the intake
  intake.moveVelocity(100);

  //launch the first acorn
  chassis->moveDistance(1500_mm);
  chassis->turnAngle(90_deg);
  chassis->moveDistance(300_mm);
  launch();
  //launch the second acorn
  chassis->turnAngle(180_deg);
  chassis->moveDistance(600_mm);
  launch();

  // grab the team acorn
  chassis->moveDistance(300_mm);
  chassis->turnAngle(90_deg);
  chassis->moveDistance(300_mm);
  launch();

}
void defensive(){
  chassis->moveDistance(500_mm);
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  
	chassis->getModel()->setBrakeMode(AbstractMotor::brakeMode::coast); // chassis braking mode
  
  pros::lcd::initialize(); // set up screen

  chassis->setState(startingState);
  
  selector::init(); // initialize the selector library
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
	pros::lcd::set_text(1, "Autonomous");
  // test to see if buttons work
  if(selector::auton == 0){
    pros::lcd::set_text(2, "Skills: Match Load Auto Launch");
  }
  if(selector::auton == 1){ 
    pros::lcd::set_text(2, "Red Offense");
    startingState = OdomState{1000_mm, -1400_mm, 0_deg};
    goalLocation = Point{1200_mm, 0_mm};
    matchLoadZone1 = Point{-1400_mm, 1400_mm};
    matchLoadZone2 = Point{-1400_mm, -1400_mm};
  }
  if(selector::auton == 2){ 
    pros::lcd::set_text(2, "Red Defense");
    startingState = OdomState{-1000_mm, -1400_mm, 0_deg};
    goalLocation = Point{1200_mm, 0_mm};
    matchLoadZone1 = Point{-1400_mm, 1400_mm};
    matchLoadZone2 = Point{-1400_mm, -1400_mm};
  }
  if(selector::auton == -1){
    pros::lcd::set_text(2, "Blue Offense");
    startingState = OdomState{-1000_mm, 1400_mm, 180_deg};
    goalLocation = Point{-1200_mm, 0_mm};
    matchLoadZone1 = Point{1400_mm, 1400_mm};
    matchLoadZone2 = Point{1400_mm, -1400_mm};
  }
  if(selector::auton == -2){
    pros::lcd::set_text(2, "Blue Defense");
    startingState = OdomState{1000_mm, 1400_mm, 180_deg};
    goalLocation = Point{-1200_mm, 0_mm};
    matchLoadZone1 = Point{1400_mm, 1400_mm};
    matchLoadZone2 = Point{1400_mm, -1400_mm};
  }
   // do the actual auton
  if(selector::auton == 1 || selector::auton == -1){
    offensive();
  }
  if(selector::auton == 2 || selector::auton == -2){
    defensive();
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
  chassis->getModel()->setBrakeMode(AbstractMotor::brakeMode::coast);
  ControllerButton runCat(ControllerDigital::L1);
  ControllerButton manRunCan(ControllerDigital::L2);
  ControllerButton runIntakeIn(ControllerDigital::R1);
  ControllerButton runIntakeOut(ControllerDigital::R2);
  ControllerButton IntakeStop(ControllerDigital::X);
  ControllerButton moveMatchLoadArm(ControllerDigital::Y);

  catapult.setBrakeMode(AbstractMotor::brakeMode::coast);

  bool isIntakeRunning = false;
  // tank drive
  while (true) {
    chassis->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY),
                                  controller.getAnalog(ControllerAnalog::rightY));
        // run the catapult when L1 is pressed
        if (runCat.changedToPressed()) {
          launch();
        }
        // run intake when R1 is not pressed
        if(runIntakeIn.changedToPressed()){
          if(!isIntakeRunning){
            intake.moveVelocity(600);
            isIntakeRunning = false;
          }
          else{
            intake.moveVelocity(0);
          }
        }
        if(runIntakeOut.changedToPressed()){
          if(!isIntakeRunning){
            intake.moveVelocity(-150);
            isIntakeRunning = false;
          }
          else{
            intake.moveVelocity(0);
          }

        }
        if(manRunCan.changedToPressed()){
          launch(1, true);
        }
        if(IntakeStop.changedToPressed()){
          intake.moveVelocity(0);
        }

        if(moveMatchLoadArm.changedToPressed()){
          matchLoadArm.set_value(!matchLoadArmState);
          matchLoadArmState = !matchLoadArmState;

        }
        

    pros::delay(20);
    
  }

}

