#include "main.h"

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
    // blue gearset, 4 in wheel diam, 5 in wheel track (center-to-center distance between the wheels (center-to-center meaning the width between the centers of both wheels))
    .withDimensions(AbstractMotor::gearset::blue, {{4.25_in, 13_in}, (imev5BlueTPR*84/36)})
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

// arm motor group
Motor wings1(10, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
Motor wings2(19, true, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);
MotorGroup wings ({wings1, wings2});

// catapult touch sensor (to figure out of it is down)
auto catapultLimitSwitch = ADIButton('A', false);

// intake motor
Motor intake(20, false, AbstractMotor::gearset::green, AbstractMotor::encoderUnits::degrees);

// acorn touch sensor to detect whether or not an acorn is loaded
auto acornLoad = OpticalSensor(5, OpticalSensorOutput::hue, true);

bool wingsOut = false;

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

void alternateWings(){
  wings1.setBrakeMode(AbstractMotor::brakeMode::brake);
  wings2.setBrakeMode(AbstractMotor::brakeMode::brake);

  if(!wingsOut){
    wings.moveVelocity(100);
    pros::delay(500);
    wings.moveVelocity(0);
    wingsOut = true;
  }
  else{
    wings.moveVelocity(-100);
    pros::delay(500);
    wings.moveVelocity(0);
    wingsOut = false;
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
  
  // set match load arm in
  matchLoadArm.set_value(true);

  intake.setVoltageLimit(12000);

  wings.setBrakeMode(AbstractMotor::brakeMode::hold);

  wings.setBrakeMode(AbstractMotor::brakeMode::brake);

  
  //autonomous();
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
	if(!catapultLimitSwitch.isPressed()){
      launch(1, false); // ensure catapult at bottom
  }

  // SECTION: MATCH LOAD BALL

  chassis->setMaxVelocity(200);


  chassis->moveDistance(5_ft);

  matchLoadArm.set_value(true);

  chassis->turnAngle(90_deg);

  intake.moveVelocity(100);


  chassis->turnAngle(180_deg);
  alternateWings();
  

  // chassis->moveDistance(-0.5_ft);
  /*
  pros::delay(1000);

  chassis->moveDistance(2_ft);

  alternateWings();
  */
  /*
  // SECTION: Middle Triball
  intake.moveVelocity(-100);
  chassis->moveDistance(4_ft);
  intake.moveVelocity(0);
  chassis->turnAngle(180_deg);
  chassis->moveDistance(4_ft);
  intake.moveVelocity(100);

  */
  /* FOR GETTING CORNER
  chassis->turnAngle(-45_deg);
  alternateWings();
  chassis->moveDistance(2_ft);
  chassis->turnAngle(45_deg);
  chassis->moveDistance(2_ft);
  */

  // int autonSelection = 0;
  // // For our purposes, offensive is when we are on the side with our goal, and defensive is when we are on the side of the other teams goal.
  
  // launch(1,false);

  // if(autonSelection == 0){
  //   pros::lcd::set_text(2, "Skills: Match Load Auto Launch");
  //   //put down match load bar
  //   matchLoadArm.set_value(true);
  //   pros::lcd::set_text(2, "Part 1");

  //   intake.moveVelocity(-100);

  //   chassis->moveRaw(600);
  //   // chassis->moveDistance(8_ft);
  //   //turn left 45 degrees
  //   // chassis->turnAngle(900_deg);
    
    
    
  //   launch(15,false);
  // }
  // if(autonSelection == 1){ 
  //   pros::lcd::set_text(2, "Red Offense");
    
  //   //put down match load bar
  //   matchLoadArm.set_value(true);

  //   //turn left 45 degrees
  //   chassis->turnAngle(-45_deg);

  //   intake.moveVelocity(100);
    
  //   launch(15,false);
  // }
  // if(autonSelection == 2){ 
  //   pros::lcd::set_text(2, "Red Defense");
    
  //   //put down match load bar
  //   matchLoadArm.set_value(true);


  //   // turn right 90 degrees
  //   chassis->turnAngle(90_deg);
  //   // move forward 1.5 meters
  //   chassis->moveDistance(1500_mm);

  //   //turn left 45 degrees
  //   chassis->turnAngle(-45_deg);

  //   intake.moveVelocity(100);

  //   launch(15,false);

  // }
  // if(autonSelection == -1){
  //   pros::lcd::set_text(2, "Blue Offense");

  //   //put down match load bar
  //   matchLoadArm.set_value(true);


  //   // turn right 90 degrees
  //   chassis->turnAngle(90_deg);
  //   // move forward 1.5 meters
  //   chassis->moveDistance(1500_mm);

  //   //turn left 45 degrees
  //   chassis->turnAngle(-45_deg);

  //   intake.moveVelocity(100);

  //   launch(15,false);
  // }
  // if(autonSelection == -2){
  //   pros::lcd::set_text(2, "Blue Defense");

  //   //put down match load bar
  //   matchLoadArm.set_value(true);

  //   //turn left 45 degrees
  //   chassis->turnAngle(-45_deg);

  //   intake.moveVelocity(100);
    
  //   launch(15,false);
  // }
  
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
  ControllerButton moveMatchLoadArm(ControllerDigital::Y);
  ControllerButton alternateWingsButton(ControllerDigital::up);
  catapult.setBrakeMode(AbstractMotor::brakeMode::coast);
  launch();
  

  bool matchLoadArmState = false;
  // tank drive
  while (true) {
    chassis->getModel()->tank(controller.getAnalog(ControllerAnalog::leftY),
                                  controller.getAnalog(ControllerAnalog::rightY));
        // run the catapult when L1 is pressed
        if (runCat.changedToPressed()) {
          launch(1, false);
        }
        if(runIntakeIn.isPressed()){
          intake.moveVelocity(600);
        }
        if(runIntakeIn.changedToReleased()){
          intake.moveVelocity(0);
        }

        if(runIntakeOut.isPressed()){
          intake.moveVelocity(-150);
        }
        if(runIntakeOut.changedToReleased()){
          intake.moveVelocity(0);
        }
        if(manRunCan.changedToPressed()){
          catapult.moveVelocity(-100);
        }
        if(manRunCan.changedToReleased()){
          catapult.moveVelocity(0);
          launch(1,false);
        }

        if(moveMatchLoadArm.changedToPressed()){
          matchLoadArm.set_value(!matchLoadArmState);
          matchLoadArmState = !matchLoadArmState;
        }
        if(alternateWingsButton.changedToPressed()){
          alternateWings();
        }

    pros::delay(20);
    
  }

}

