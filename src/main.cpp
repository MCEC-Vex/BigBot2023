#include "main.h"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include <math.h>


pros::Motor topLeft(11, false);
pros::Motor topRight(1, true);
pros::Motor botLeft(18, true);
pros::Motor botRight(20, false);

pros::Motor Catapult(17);
pros::Motor arm(14); 
pros::Motor intake(19);

//pros::Motor Catapult();   //add port

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
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
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
 * 
 * 
 * 
 */



void autonomous() {

	pros::lcd::set_text(1, "THIS IS AUTON!");

	// BASIC MOVEMENTS
	topLeft.move_velocity(-25);   //forward 
	topRight.move_velocity(-25);
	botLeft.move_velocity(25);
	botRight.move_velocity(25);
	pros::delay(5000); //5 sec
	topLeft.move_velocity(0);
	topRight.move_velocity(0);
	botLeft.move_velocity(0);
	botRight.move_velocity(0);

	topLeft.move_velocity(25);  //backwards 
	topRight.move_velocity(25);
	botLeft.move_velocity(-25);
	botRight.move_velocity(-25);
	pros::delay(5000);
	topLeft.move_velocity(0);
	topRight.move_velocity(0);
	botLeft.move_velocity(0);
	botRight.move_velocity(0);

	topLeft.move_velocity(25);  //turn- counter clockwise 
	topRight.move_velocity(-25);
	botLeft.move_velocity(-25);
	botRight.move_velocity(25);
	pros::delay(5000);
	topLeft.move_velocity(0);
	topRight.move_velocity(0);
	botLeft.move_velocity(0);
	botRight.move_velocity(0);

	topLeft.move_velocity(-25);  //turn- counter clockwise 
	topRight.move_velocity(25);
	botLeft.move_velocity(25);
	botRight.move_velocity(-25);
	pros::delay(5000);
	topLeft.move_velocity(0);
	topRight.move_velocity(0);
	botLeft.move_velocity(0);
	botRight.move_velocity(0);

	
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

	int yMotion;
	int xMotion;

	while (true)
	{

		pros::lcd::set_text(1, std::to_string(topLeft.get_position()));
		pros::lcd::set_text(2, std::to_string(topRight.get_position()));
		pros::lcd::set_text(3, std::to_string(botLeft.get_position()));
		pros::lcd::set_text(4, std::to_string(botRight.get_position()));

		pros::Controller master(pros::E_CONTROLLER_MASTER);
		// driving control code

		yMotion = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); // ik this looks wrong, but it works
		xMotion = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);


		int right = -xMotion + yMotion; //-power + turn
		int left = xMotion + yMotion;	// power + turn

		topLeft.move(left); // Swap negatives if you want the bot to drive in the other direction
		botLeft.move(-left);
		botRight.move(right);
		topRight.move(-right);


		if (master.get_digital(DIGITAL_R2))
		{ // runs flywheel while holding R2 down

			Catapult.move_velocity(30); // Changes flywheel velocity
			pros::lcd::set_text(5, std::to_string(Catapult.get_actual_velocity()));
		}
		else if(master.get_digital(DIGITAL_UP))
		{
			arm.move_velocity(30);
			pros::lcd::set_text(5, std::to_string(arm.get_actual_velocity()));
		}
		else if(master.get_digital(DIGITAL_DOWN))
		{
			arm.move_velocity(30);
			pros::lcd::set_text(5, std::to_string(arm.get_actual_velocity()));
		}
		else if(master.get_digital(DIGITAL_L1))
		{
			intake.move_velocity(30);
			pros::lcd::set_text(5, std::to_string(intake.get_actual_velocity()));

		}


		pros::delay(20);
	}
}
