#include "main.h"
#include "pros/misc.h"
#include "pros/motors.hpp"
#include <math.h>
#include <string>
#include <vector>
#include "okapi/api.hpp"
#include "okapi/api/chassis/controller/chassisControllerPid.hpp"
using namespace okapi;
pros::Motor Catapult(17, false);
pros::Motor Arm(20, false);
pros::Motor Intake(18, false);
pros::Rotation RotationSensor(19);
pros::ADIDigitalOut Piston('A');
pros::Imu imu_sensor(10);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.


/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */

void on_center_button()
{
	static bool pressed = false;
	pressed = !pressed;
	if (pressed)
	{
		pros::lcd::set_text(2, "I was pressed!");
	}
	else
	{
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
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


// This is PID 
// !Don't Touch it
	std::shared_ptr<ChassisController> bot = ChassisControllerBuilder()
	 .withMotors(13, -16, -14, 11) // front right and back right were reversed in order to go forward
	 // change P then D first then I only if necessary
	 // start with P I and D with zero
	 .withGains( // 0.7, 0, 0.1 results: faster, shaking less violently 0
		 // 0.5 =
		 {0.0006, 0,0},	  // Distance controller gains
		 {0.001, 0, 0},	  // turn controller gains
		 {0.0, 0, 0.0000} // Angle controller (helps bot drive straight)
		 )
	 .withMaxVelocity(200)
	 // Green gearset, 3 inch wheel diam, 9 inch wheel track
	 .withDimensions(AbstractMotor::gearset::green, {{4_in, 14.8_in}, imev5GreenTPR})
	 .build();

// This set the Catapult's Position
void setCatapult(){
	// Set the Rotation Sensor
	pros::Rotation RotationSensor(19);

	// * Run until the Catault is at the desired position which is Down
	while (true)
	{
		// * The Rotation Sensor has value with no points so if it's 360.00, it's 36000
		// If the Rotation Sensor is less than 33998
		if(RotationSensor.get_angle() < 33998)
		{
			// Move the Catapult down
			Catapult.move_velocity(200);
		}
		else
		{
			// Stop the Catapult
			Catapult.move_velocity(0);
			// Break the loop
			break;
		}
	}
}

// * This function is to get the ball into the Catapult
void getBall(){
	setCatapult(); // Set the catapult
	Intake.move_velocity(-200); // The intake starts
	Arm.move_absolute(700, 200); // The Arm for intake goes down
	pros::delay(500); // Wait for the arm to go down
	for (int i = 0; i >= -1500; i = i - 500) // Slowly (exponentially) move the arm up
	{
		Intake.move_velocity(-100);
		Arm.move_absolute(i, 200);
		pros::delay(250);
	}
}

// * This function is to turn the bot to the left
void negativeTurn(int degrees)
{
	// !Reset's the IMU's rotation 
    imu_sensor.tare_rotation();

	bot->turnAngle(-degrees*1_deg); // Turn the bot to the left

	// * Solve the greater Error - so If the error is -40 degrees, ask the bot to turn -40 degrees
	if(imu_sensor.get_rotation() >= -degrees)
	{
		int value = degrees - abs(imu_sensor.get_rotation()); // Gets The error
		bot->turnAngle(-round(value)*1_deg); // Turns it
	}

	// * Solve minor error, So we turned the bot -40 degrees in above examples 
	// * but bot is not accurate so it just turned -30 degrees 
	// * so until it's not greater than or equal to desired degrees, change -10 more degrees
	while(imu_sensor.get_rotation() >= -degrees)
	{
    	bot->turnAngle(-10_deg); // Turn the bot 10 degrees to Left
		if(imu_sensor.get_rotation() <= -(degrees)) // break the loop when it's greatar than it
		{
			break;
		}
	}
}

// This function is to turn the bot to the right
void positiveTurn(int degrees)
{
	// !Reset's the IMU's rotation 
	imu_sensor.tare_rotation();
	setCatapult(); // Set the catapult
	bot->turnAngle(degrees*1_deg); // Turn the bot to the right

	// * Solve the greater Error - so If the error is 40 degrees, ask the bot to turn 40 degrees
	if(round(imu_sensor.get_rotation()) <= degrees)
	{
		int value = abs(imu_sensor.get_rotation()) - degrees; // Gets The error
		bot->turnAngle(-round(value)*1_deg); // Turns it
	}

	// * Solve minor error, So we turned the bot 40 degrees in above examples 
	// * but bot is not accurate so it just turned 30 degrees 
	// * so until it's not greater than or equal to desired degrees, change 10 more degrees
	while(round(imu_sensor.get_rotation()) <= degrees)
	{
    	bot->turnAngle(10_deg); // Turn the bot 10 degrees to Right
		if(imu_sensor.get_rotation() >= degrees) // break the loop when it's greatar than it
		{
			break;
		}
	}
}

void redAuton() {

	// * Part 1 - Set the Catapult, move forward, turn to the left
	setCatapult();
	bot->moveDistance(24_in);
	negativeTurn(50);

	// * Part 2 - Move forward, Get the ball and turn to the left for launching
	bot->moveDistance(19_in);
	getBall();
	negativeTurn(95);

	// * Part 3 - Launch the ball, have a delay and turn to the right to get another ball
	Catapult.move_velocity(200); // The catapult goes down
	pros::delay(500); // Wait for launch
	positiveTurn(85);

	// * Part 4 - Move forward, get the ball and turn to the left for launching
	bot->moveDistance(10_in);
	getBall();
	pros::delay(500); // Wait for launch
	negativeTurn(80);

	// * Part 5 - Launch the ball, have a delay and turn to the right to get another ball
	Catapult.move_velocity(300); // The catapult goes down
	pros::delay(500); // Wait for launch
	Catapult.move_velocity(0); // The catapult stops

	// * Part 6 - Move backwards, have a delay and move foward for THE SLAPP TO THE BALL
	positiveTurn(80);
	pros::delay(100);
	bot->moveDistance(-52_in);

	negativeTurn(50);


	pros::delay(1000); // Wait for launch
	pros::lcd::set_text(5, std::to_string('The Program ends'));
}
void blueAuton() {

	// * Part 1 - Set the Catapult, move forward, turn to the left
	setCatapult();
	bot->moveDistance(24_in);
	positiveTurn(50);

	// * Part 2 - Move forward, Get the ball and turn to the left for launching
	bot->moveDistance(19_in);
	getBall();
	positiveTurn(95);

	// * Part 3 - Launch the ball, have a delay and turn to the right to get another ball
	Catapult.move_velocity(200); // The catapult goes down
	pros::delay(500); // Wait for launch
	negativeTurn(85);

	// * Part 4 - Move forward, get the ball and turn to the left for launching
	bot->moveDistance(10_in);
	getBall();
	pros::delay(500); // Wait for launch
	positiveTurn(80);

	// * Part 5 - Launch the ball, have a delay and turn to the right to get another ball
	Catapult.move_velocity(300); // The catapult goes down
	pros::delay(500); // Wait for launch
	Catapult.move_velocity(0); // The catapult stops

	// * Part 6 - Move backwards, have a delay and move foward for THE SLAPP TO THE BALL
	negativeTurn(80);
	pros::delay(100);
	bot->moveDistance(-52_in);

	positiveTurn(50);


	pros::delay(1000); // Wait for launch
	pros::lcd::set_text(5, std::to_string('The Program ends'));
}

void slapBall(){
	for(int i=0; i<= 8; i++){
	bot->moveDistance(-5_in);
	bot->moveDistance(5_in);
	}
}

void autonomous()
{
	pros::Rotation RotationSensor(19);

	pros::lcd::set_text(1, "THIS IS AUTON!");

	// // ! Call the redAuton function when needed
	// redAuton();

	// // ! Call the blueAuton function when needed
	// // TODO: Code the blueAuton function

	bot->moveDistance(2_ft);
   
	// blueAuton();
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
void opcontrol()
{

	pros::Motor FrontLeft(13, false);  //counterclockwise
	pros::Motor FrontRight(16, true);  //! clockwise   
	pros::Motor BackLeft(11, true);  //!clockwise 
	pros::Motor BackRight(14, false);  //counterclockwise 
	pros::Motor MidRight(15, false);  //
	pros::Motor MidLeft(12, false);
	pros::Motor Catapult(17, false);
	pros::Motor Arm(20, false);
	pros::Motor Intake(18, false);
	pros::Rotation RotationSensor(19);
	pros::ADIDigitalOut Piston('A');
	pros::Imu imu_sensor(10);
	// imu_sensor.reset();



	pros::lcd::set_text(1, "READY TO DRIVE");
	int yMotion;
	int xMotion;
	int value;

	pros::Controller master(pros::E_CONTROLLER_MASTER);
	Catapult.tare_position();
	Arm.tare_position();

	//-33 Arm encoder units for intake


	imu_sensor.tare_yaw();
	imu_sensor.tare_roll();
	imu_sensor.tare_pitch();

	while (true)
	{

	pros::lcd::set_text(1, "YAW: " + std::to_string(imu_sensor.get_yaw()));
	pros::lcd::set_text(2, "PITCH: " + std::to_string(imu_sensor.get_roll()));
	pros::lcd::set_text(3, "ROLL: " + std::to_string(imu_sensor.get_pitch()));

	// pros::lcd::set_text(2, "IMU quaternion y: " + std::to_string(qt.y));
	// pros::lcd::set_text(3, "IMU quaternion z: " + std::to_string(qt.z));
	// pros::lcd::set_text(4, "IMU quaternion w: " + std::to_string(qt.w));

		// pros::lcd::set_text(1, "Arm:" + std::to_string(Arm.get_position()));
		// pros::lcd::set_text(2, "Front Right Motor:" + std::to_string(FrontRight.get_position()));
		// pros::lcd::set_text(3, "Back Left Motor:" + std::to_string(BackLeft.get_position()));
		// pros::lcd::set_text(4, "Back Right Motor:" + std::to_string(BackRight.get_position()));
		// pros::lcd::set_text(5, "Rotation Sensor: " + std::to_string(RotationSensor.get_position()));
		// pros::lcd::set_text(6, "Catapult: " + std::to_string(Catapult.get_position()));


		// driving control code

		xMotion = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X); // ik this looks wrong, but it works
		yMotion = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);

		int right = -yMotion + xMotion; //-power + turn
		int left = yMotion + xMotion;	// power + turn

		pros::lcd::set_text(3, "Y:" + std::to_string(yMotion)); //Y-motion pos=up neg=down
		pros::lcd::set_text(4, "X: " + std::to_string(xMotion));//X-motion pos=clockwise  neg=counterclockwise
		pros::lcd::set_text(5, "right" + std::to_string(right)); 
		pros::lcd::set_text(6, "left" + std::to_string(left));
		// Test.move(right);   //clockwise when true //counterclockwise when false 

		FrontLeft.move(left); // Swap negatives if you want the bot to drive in the other direction
		BackLeft.move(-left);
		MidLeft.move(yMotion); //Power
		MidRight.move(-yMotion); //Power 
		BackRight.move(right);
		FrontRight.move(-right);


		if (master.get_digital(DIGITAL_R1))
		{
			RotationSensor.set_data_rate(0);
			Catapult.move_velocity(200);
			pros::lcd::set_text(5, std::to_string(RotationSensor.get_angle()));
		}
		else if (RotationSensor.get_angle() <= 33998)
		{ // 3050 ->2700(no data rate)-> 3565 (data rate)
			Catapult.move_velocity(200);
		}
		// value to launch (34000) is value due to catapult starting position not being zero
		else
		{
			// RotationSensor.set_data_rate(55);
			Catapult.move_velocity(0);
		}

		if (master.get_digital(DIGITAL_L1))
		{
			Arm.move_velocity(-112);
			pros::lcd::set_text(5, "Arm Velocity:" + std::to_string(Arm.get_actual_velocity()));
		}
		else if (master.get_digital(DIGITAL_L2))
		{
			Arm.move_velocity(112);
			pros::lcd::set_text(5, "Arm Velocity:" + std::to_string(Arm.get_actual_velocity()));
		}
		else
		{
			Arm.move_velocity(0);
		}

		if (master.get_digital(DIGITAL_R2))
		{
			Intake.move_velocity(-200);
			pros::lcd::set_text(5, "Intake Velocity:" + std::to_string(Intake.get_actual_velocity()));
		}
		else if (master.get_digital(DIGITAL_DOWN))
		{
			Intake.move_velocity(200);
			pros::lcd::set_text(5, "Intake Velocity:" + std::to_string(Intake.get_actual_velocity()));
		}
		if (master.get_digital(DIGITAL_A))
		{
			Piston.set_value(false);
		}
		else
		{
			Piston.set_value(true);
		}

		pros::delay(20);
	}
}
