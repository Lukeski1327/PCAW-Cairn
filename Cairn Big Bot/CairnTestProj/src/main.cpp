//Includes
#include "main.h"
#include "lemlib/api.hpp"

/*  Optical Sensor Declaration : the number is the port it's plugged into
    This is the color sensor and will be used to detect the opponent's rings so they can be thrown out of the bot */
pros::Optical optical_sensor(21);

pros::Motor elevatorMotor1(10, pros::v5::MotorGears::blue);
pros::Motor intakeMotor(16);
char teamColor = 'n';

/*  Odometry Sensor Declarations */
pros::Rotation leftWheelRot(15);
pros::Rotation rightWheelRot(17);
pros::Rotation horizWheelRot(15);
pros::IMU IMU(14);
lemlib::TrackingWheel VertTrackingWheelLeft(&leftWheelRot, lemlib::Omniwheel::OLD_325, 1);
lemlib::TrackingWheel VertTrackingWheelRight(&rightWheelRot, lemlib::Omniwheel::OLD_325, 1);
lemlib::TrackingWheel horiTrackWheel(&horizWheelRot, lemlib::Omniwheel::NEW_275_HALF, 1);

/*  Solenoid Declarations: pneumatic component control */
pros::ADIDigitalOut sol1 ('G', LOW);

/*	Motor Group Declaration : the number are the ports they are in, and the minus '-' means they are reversed
	Typically only the right side needs to be reversed */
pros::MotorGroup left_motor_group({ 18, 19, -20 });
pros::MotorGroup right_motor_group({ 13, -12, 11 });

/*  Controller declaration, it's almost always this exact line of code but is still needed
    Note that if you want to use two controllers, you need to declare a second controller with a differend name */
pros::Controller controller(pros::E_CONTROLLER_MASTER);

/*  Drive Train is the sum of the two motor groups, their track width, 
    the type and size of wheels used, the rpm of the drive train, and the horizontal drift
    If any of the above needs further defining check the lemlib docs here: 
        https://lemlib.readthedocs.io/en/v0.5.0/tutorials/2_configuration.html
*/
lemlib::Drivetrain drivetrain(&left_motor_group, // left motor group
                              &right_motor_group, // right motor group
                              9, // 9 inch track width
                              lemlib::Omniwheel::OLD_325, // using new 4" omnis
                              200, // drivetrain rpm is 200
                              2 // horizontal drift is 2 (for now)
);

// auton functions
void autoMove(int velocity, int delayms) {
    left_motor_group.move_velocity(velocity);
    right_motor_group.move_velocity(velocity);
    pros::delay(delayms);
    left_motor_group.brake();
    right_motor_group.brake();
    pros::delay(100);
}

void autoTurn(int velocity, int heading, bool turnLeft) {
    
    switch(turnLeft) {
        case true:
            while(IMU.get_heading() > heading) {
                left_motor_group.move_velocity(-velocity);
                right_motor_group.move_velocity(velocity);
                std::string IMUstr = std::to_string(IMU.get_heading());
                controller.set_text(0, 0, IMUstr);
            }            
            break;
        case false:
            while(IMU.get_heading() < heading) {
                left_motor_group.move_velocity(velocity);
                right_motor_group.move_velocity(-velocity);
                std::string IMUstr = std::to_string(IMU.get_heading());
                controller.set_text(0, 0, IMUstr);
            }
            break;
        default:
            break;
    }
    left_motor_group.brake();
    right_motor_group.brake();
    pros::delay(100);
}

void autoPneumatics(bool signal) {
    switch (signal)
    {
    case true:
        sol1.set_value(LOW);
        pros::delay(100);
        break;
    case false:
        sol1.set_value(HIGH);
        pros::delay(100);
        break;
    default:
        break;
    }
}

void autoMoveAndGrab(int velocity, int delayms, int grabDelay) {
    int finalMoveDelay = delayms-grabDelay;

    left_motor_group.move_velocity(velocity);
    right_motor_group.move_velocity(velocity);
    pros::delay(grabDelay);
    autoPneumatics(true);
    pros::delay(finalMoveDelay);
    left_motor_group.brake();
    right_motor_group.brake();
    pros::delay(100);
}

/*  lateral PID controller
     don't worry about tuning this yet, if you want more info check the lemlib or PROS docs for PID related info */
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

/*  angular PID controller
    don't worry about tuning this yet, if you want more info check the lemlib or PROS docs for PID related info */
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

/*  Odom Sensor Group
    all null for now since we have no sensors, if you want more info on Odom check the lemlib or PROS docs for any
    odom related articles or docs
*/
lemlib::OdomSensors sensors(&VertTrackingWheelLeft, // left vertical tracking wheel
                            &VertTrackingWheelRight, // right vertical tracking wheel
                            &horiTrackWheel, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr bc we don't have a second one
                            &IMU // inertial sensor
);

/*  Chassis Declaration
    sum of the drive train, PID controllers, and odom sensor array*/
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
    optical_sensor.set_led_pwm(100);
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
    // pros::delay(100);
    // autoMove(-600, 1000);
    // autoPneumatics(true);
    // autoTurn(300, 500, false);

    // autoMoveAndGrab(600, 1500, 750);
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
		pros::lcd::print(5, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

        int leftY;
        int rightY;
        int leftX;
        int rightX;

        leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // pros::lcd::print(0, "%f", chassis.getPose().x);
        // pros::lcd::print(1, "%f", chassis.getPose().y);
        // pros::lcd::print(2, "%f", chassis.getPose().theta);
        pros::lcd::print(3, "%f", IMU.get_heading());
        pros::lcd::print(4, "%f", IMU.get_accel());
        pros::lcd::print(0, "%c", teamColor);

        if(controller.get_digital(DIGITAL_L1)) {
            elevatorMotor1.move(-127);
            intakeMotor.move(-127);
        } else if(controller.get_digital(DIGITAL_L2)) {
            elevatorMotor1.move(127);
            intakeMotor.move(127);
        } else {
            elevatorMotor1.brake();
            intakeMotor.brake();
        }

        // below is experimentation with the optical sensor
        int detectedColor = optical_sensor.get_hue();
        std::string redOrBlue = "";
        pros::lcd::print(6, "Optical Sensor Hue: %d", detectedColor);

        switch(teamColor) {
            case 'r': // red team
                if(detectedColor > 170 && detectedColor < 210) {
                    pros::lcd::print(7, "Blue detected");
                    elevatorMotor1.move_relative(500, 600);
                    pros::delay(250);
                    elevatorMotor1.move_relative(-50, 600);
                }
                pros::lcd::print(1, "Red Team Selected");
                break;
            case 'b': // blue team
                if(detectedColor > 0 && detectedColor < 20) {
                    pros::lcd::print(7, "Red detected");
                    elevatorMotor1.move_relative(500, 600);
                    pros::delay(250);
                    elevatorMotor1.move_relative(-50, 600);
                }
                pros::lcd::print(1, "Blue Team Selected");
                break;
            default: // not selected/disabled
                pros::lcd::clear_line(7);
                pros::lcd::print(1, "No Team Selected");
                break;
        }
        
        // Tank Control
        // chassis.tank(leftY, rightY);

        // Split Arcade/Drone Control
        chassis.arcade(leftY, rightX);

        // pneumatic control
        if(controller.get_digital(DIGITAL_DOWN)) {
            sol1.set_value(LOW);
        } else if(controller.get_digital(DIGITAL_UP)) {
            sol1.set_value(HIGH);
        }

        if(controller.get_digital(DIGITAL_A)) {
            teamColor = 'r';
        } else if(controller.get_digital(DIGITAL_B)) {
            teamColor = 'b';
        } else if(controller.get_digital(DIGITAL_X)) {
            teamColor = 'n';
        }

		pros::delay(20);
	}
}