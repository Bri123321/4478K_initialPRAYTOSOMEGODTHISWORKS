#include "main.h"
#include "api.h"
#include "okapi/api.hpp"
#include "pros/apix.h"
using namespace okapi;

// declares ports
pros::Controller master(CONTROLLER_MASTER);
Controller Controller1;

Motor mCatapult(1);	//catapult declaration
Motor mIntakeL(20);	//intake declaration
Motor mIntakeR(11);
ADIButton catapultLimit('A');

// Declares the chassis.
std::shared_ptr<ChassisController> Kenneth = 
	ChassisControllerBuilder()
		.withMotors(
			{16, 9}, //left motors
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
		.withGains(
			{1, 0, 0}, // distance gains(constants)
			{1, 0, 0} // turning gains(constants)
			)
		.build();


// Declares the catapult
const double liftkP = 0.001;
const double liftkI = 0.0001;
const double liftkD = 0.0001;

std::shared_ptr<AsyncPositionController<double, double>> catapultController = 
  AsyncPosControllerBuilder()
    .withMotor(1)
    .withGains({liftkP, liftkI, liftkD})
    .build();

/*
* A callback function for LLEMU's center button.
*
* When this callback is fired, it will toggle line 2 of the LCD text between
* "I was pressed!" and nothing.
*/
int selection = 0;
void autonSelector() {
	if(selection<=2){
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
	}
	
}

void intake(int speed, float rotations){
	mIntakeL.moveAbsolute(rotations, speed);
	mIntakeR.moveAbsolute(rotations, speed);
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
	 
}

void test_auto(){
	Kenneth->moveDistance(12_in); // for testing PIDs
}

void leftWPAuto(){
	//moves to middle of field
	Kenneth->moveDistance(36_in);

	//turns and moves to goal
	Kenneth->turnAngle(-90_deg);		//is left or right negative? test.
	Kenneth->moveDistance(12_in);

	//outtakes triball into goal
	intake(100, 3);

	//pushes triballs over the bar
	Kenneth->moveDistance(-24_in);

	//moves to the load zone
	Kenneth->moveDistance(12_in);
	Kenneth->turnAngle(-45_deg);
	Kenneth->moveDistance(68_in);

	//intakes triball in load zone
	intake(100, -3);

	//turn to hit bar
	Kenneth->turnAngle(135_deg);
	Kenneth->moveDistance(48_in);

	//outtake triball
	intake(100, 3);	
}

void rightWPAuto(){
	//moves to middle of field
	Kenneth->moveDistance(36_in);

	//turns and moves to goal
	Kenneth->turnAngle(90_deg);
	Kenneth->moveDistance(12_in);

	//outtakes triball into goal
	intake(100, 3);

	//turns and intakes nearest triball on the center line 
	Kenneth->turnAngle(180_deg);
	intake(100, 3);
	Kenneth->moveDistance(24_in);

	//turns and deposits it in goal
	Kenneth->turnAngle(180_deg);
	intake(100, 3);
	Kenneth->moveDistance(24_in);

	//turns and intakes the further triball on the center line
	Kenneth->turnAngle(180_deg);
	Kenneth->moveDistance(48_in);
	intake(100, 3);

	//turns and deposits the triball in the goal
	Kenneth->turnAngle(180_deg);
	Kenneth->moveDistance(48_in);
	intake(100, 3);

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
		} else if (rightY < 0 && leftY < 0) {
			rightY = (rightY * rightY) / -127;
			leftY = (leftY * leftY)  / -127;
			Kenneth->getModel()->tank(rightY, leftY);
		} else {
			Kenneth->getModel()->tank(Controller1.getAnalog(ControllerAnalog::leftY), Controller1.getAnalog(ControllerAnalog::rightY));
		}*/
		Kenneth->getModel()->tank(Controller1.getAnalog(ControllerAnalog::leftY), Controller1.getAnalog(ControllerAnalog::rightY));
		
		ControllerButton intakeInButton(ControllerDigital::R1);
		ControllerButton intakeOutButton(ControllerDigital::L1);
		ControllerButton catapultDownButton(ControllerDigital::B);
		ControllerButton catapultUpButton(ControllerDigital::A);

		if (intakeInButton.isPressed()){		//intake movements with 1st left and right triggers
			mIntakeL.moveVelocity(127);		//
			mIntakeR.moveVelocity(-127);
		}
		else if (intakeOutButton.isPressed()){	
			mIntakeL.moveVelocity(-127);
			mIntakeR.moveVelocity(127);
		}
		else{
			mIntakeL.moveVelocity(0);
			mIntakeR.moveVelocity(0);
		}

		if (catapultDownButton.changedToPressed()){
			mCatapult.moveVelocity(-40);
		}
		else if(catapultUpButton.changedToPressed()){
			mCatapult.moveVelocity(127);
		}

		if (catapultLimit.isPressed()){
			mCatapult.moveVelocity(0);
		}

		pros::delay(20);
	}
}
