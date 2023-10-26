#include "main.h"
#include "api.h"
#include "okapi/api.hpp"
#include "pros/apix.h"
#include "string.h"
using namespace okapi;

// declares ports
pros::Controller master(CONTROLLER_MASTER);
Controller Controller1;

Motor mCatapult(2);	//catapult declaration
Motor mIntakeL(19);	//intake declaration 
Motor mIntakeR(12);
ADIButton catapultLimit('A');

// Declares the chassis.
std::shared_ptr<ChassisController> Kenneth = 
	ChassisControllerBuilder()
		.withMotors(
			{16, 9,},
			{-15, -14} // right motors
			) //uses motors 17-20
		//blue motors, 3.25 in diameter, 9 in apart
		.withDimensions({AbstractMotor::gearset::blue, (36.0/48.0)}, {{3.25_in, 9_in}, imev5BlueTPR})
		.withLogger(
		std::make_shared<Logger>(
			TimeUtilFactory::createDefault().getTimer(), //Timer
			"/ser/sout", // Output to Pros terminal 
			Logger::LogLevel::debug //Most verbose log level
		)
		)
		/*.withGains(
			{1.35, 0.0, 0.25}, // distance gains(constants)
			{1.35, 0.25, 0} // turning gains(constants)
			)*/
		.build();


// Declares the catapult
const double liftkP = 0.001;
const double liftkI = 0.0001;
const double liftkD = 0.0001;

std::shared_ptr<AsyncPositionController<double, double>> catapultController = 
  AsyncPosControllerBuilder()
    .withMotor(2)
    //.withGains({liftkP, liftkI, liftkD})
    .build();

// Declares the catapult
const double intakekP = 0.1;
const double intakekI = 0;
const double intakekD = 0;



std::shared_ptr<AsyncPositionController<double, double>> intakeController = 
  AsyncPosControllerBuilder()
    .withMotor(
		{-19, 12}
	)
    //.withGains({intakekP, intakekI, intakekD})
    .build();

/*
* A callback function for LLEMU's center button.
*
* When this callback is fired, it will toggle line 2 of the LCD text between
* "I was pressed!" and nothing.
*/
int selection = 0;
void autonSelector() {
	if(selection<=3){
		selection ++;
	} else {
		selection = 0;
	}

	switch (selection) {
	case 0:
		pros::lcd::set_text(2, "Test auton");
		break;
	case 1:
		pros::lcd::set_text(2, "Left WP auton");
		break;
	case 2:
		pros::lcd::set_text(2, "Right WP auton");
		break;
	case 3:
		pros::lcd::set_text(2, "Skills auton");
		break;
	}
	
}



/*
* Runs initialization code. This occurs as soon as the program is started.
*
* All other competition modes are blocked by initialize; it is recommended
* to keep execution time for this mode under a few seconds.
*/
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Press center button to select autonomous");
	pros::lcd::register_btn1_cb(autonSelector);
	
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	pros::lcd::register_btn1_cb(autonSelector);
}

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
	intakeController->reset();
}

void test_auto(){
	int current = 0;
	intakeController->reset();
	intakeController->setTarget(current + 7000);

}

void leftWPAuto(){
	int current = 0;
	
	//moves to middle of field
	Kenneth->moveDistance(82.5_in);

	//turns and moves to goal
	Kenneth->turnAngle(-190_deg);		
	Kenneth->moveDistance(15_in);

	//outtakes triball into goal
	intakeController->setTarget(current + 5000); //outtake ~3 rotations
	intakeController->waitUntilSettled();
	Kenneth->moveDistance(-8_in);
	Kenneth->turnAngle(180_deg);
	Kenneth->setMaxVelocity(500);
	Kenneth->moveDistance(-90_in);

	Kenneth->turnAngle(185_deg);
	Kenneth->moveDistance(46_in);
	
}

void rightWPAuto(){
	int current = 0;
	//moves to middle of field
	Kenneth->moveDistance(102.5_in);

	//turns and moves to goal
	Kenneth->turnAngle(195_deg);		
	Kenneth->moveDistance(12_in);

	//outtakes triball into goal
	intakeController->setTarget(current+2000); //outtake ~3 rotations
	intakeController->waitUntilSettled();

	//turns and intakes the nearest triball on the center line
	Kenneth->moveDistance(-7_in);
	/*
	Kenneth->turnAngle(347.5_deg);
	current = 0;
	intakeController->setTarget(current+6000);
	Kenneth->moveDistance(30_in);

	//turns and deposits the triball in the goal
	Kenneth->turnAngle(-380_deg);
	Kenneth->moveDistance(48_in);
	intakeController->setTarget(current-1000);

	//turns and intakes the further triball on the center line
	Kenneth->moveDistance(-7_in);
	Kenneth->turnAngle(350_deg);
	intakeController->setTarget(current+3000);
	Kenneth->moveDistance(48_in);

	//turns and deposits the triball in the goal
	Kenneth->turnAngle(310_deg);
	Kenneth->moveDistance(48_in);
	intakeController->setTarget(current-1000);
*/
}

void skillsAuton(){
	int current = 0;
	int x = 0;
	Kenneth->setMaxVelocity(200);
	Kenneth->moveDistance(1.5_in);
	catapultController->setMaxVelocity(60);
	catapultController->setTarget(-100000000);
	/*while(x<=40){
		catapultController->setTarget(current - 180);
		x++;
	}*/
}

void intake(float speed, float revolutions){
	mIntakeL.moveAbsolute(revolutions, speed);
	mIntakeR.moveAbsolute(revolutions, speed);
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
	switch (selection) {
	case 0:
		test_auto();
		break;
	case 1:
		leftWPAuto();
		break;
	case 2:
		rightWPAuto();
		break;
	case 3:
		skillsAuton();
		break;
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
	


	while (true) {
		/*
		pros::lcd::print(0, "%d %d %d", (lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
		
		mfrontright = master.get_analog(ANALOG_RIGHT_Y);
		mbackright = master.get_analog(ANALOG_RIGHT_Y);
		mfrontleft = master.get_analog(ANALOG_LEFT_Y);
		mbackleft = master.get_analog(ANALOG_LEFT_Y);
		*/

		float rightY = Controller1.getAnalog(ControllerAnalog::rightY);
		float leftY = Controller1.getAnalog(ControllerAnalog::leftY);
		/*
		if (rightY >= 0 && leftY >=0){
			rightY = (rightY * rightY) / 127;
			leftY = (leftY * leftY) / 127;
			Kenneth->getModel()->tank(rightY, leftY);
		} else {
			Kenneth->getModel()->tank(Controller1.getAnalog(ControllerAnalog::leftY), Controller1.getAnalog(ControllerAnalog::rightY));
		}
		if (rightY < 0 && leftY < 0) {
			rightY = (rightY * rightY) / -127;
			leftY = (leftY * leftY)  / -127;
			Kenneth->getModel()->tank(rightY, leftY);
		} else {
			Kenneth->getModel()->tank(Controller1.getAnalog(ControllerAnalog::leftY), Controller1.getAnalog(ControllerAnalog::rightY));
		}*/
		Kenneth->getModel()->tank(.9*Controller1.getAnalog(ControllerAnalog::leftY), .9*Controller1.getAnalog(ControllerAnalog::rightY));
		
		ControllerButton intakeInButton(ControllerDigital::R1);
		ControllerButton intakeOutButton(ControllerDigital::L1);
		ControllerButton intakeStopButton(ControllerDigital::L2);
		ControllerButton catapultDownButton(ControllerDigital::B);
		ControllerButton catapultUpButton(ControllerDigital::A);

		if (intakeInButton.changedToPressed()){		//intake movements with 1st left and right triggers
			mIntakeL.moveVelocity(127);		//
			mIntakeR.moveVelocity(-127);
		}
		else if (intakeOutButton.changedToPressed()){	
			mIntakeL.moveVelocity(-127);
			mIntakeR.moveVelocity(127);
		}
		else if (intakeStopButton.changedToPressed()){
			mIntakeL.moveVelocity(0);
			mIntakeR.moveVelocity(0);
		}

		if (catapultDownButton.changedToPressed()){
			mCatapult.moveVelocity(-60);
		}
		else if(catapultUpButton.changedToPressed()){
			mCatapult.moveVelocity(0);
		}

		if (catapultLimit.isPressed()){
			mCatapult.moveVelocity(0);
		}

		pros::delay(20);
	}
}
