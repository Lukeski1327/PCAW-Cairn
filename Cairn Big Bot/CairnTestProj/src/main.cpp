//Includes
#include "main.h"
#include "lemlib/api.hpp"

/*  Optical Sensor Declaration : the number is the port it's plugged into
    This is the color sensor and will be used to detect the opponent's rings so they can be thrown out of the bot */
pros::Optical optical_sensor(12);

pros::Motor elevatorMotor1(13, pros::v5::MotorGears::blue);
pros::Motor elevatorMotor2(14, pros::v5::MotorGears::blue);
bool reversed = false;

/*  Odometry Sensor Declarations */
pros::Rotation leftWheelRot(15);
pros::Rotation rightWheelRot(16);
pros::Rotation horizWheelRot(17);
pros::IMU IMU(21);
lemlib::TrackingWheel VertTrackingWheelLeft(&leftWheelRot, lemlib::Omniwheel::OLD_325, 1);
lemlib::TrackingWheel VertTrackingWheelRight(&rightWheelRot, lemlib::Omniwheel::OLD_325, 1);
lemlib::TrackingWheel horiTrackWheel(&horizWheelRot, lemlib::Omniwheel::NEW_275_HALF, 1);

/*  Solenoid Declarations: pneumatic component control */
pros::ADIDigitalOut sol1 ('G', LOW);

/*	Motor Group Declaration : the number are the ports they are in, and the minus '-' means they are reversed
	Typically only the right side needs to be reversed */
pros::MotorGroup left_motor_group({ 18, -19, 20 });
pros::MotorGroup right_motor_group({ -13, 12, -11 });

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

    // pot1.calibrate();
    // testMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

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
 */
void autonomous() {

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
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

        if(controller.get_digital(DIGITAL_L1)) {
            reversed = !reversed;
            switch (reversed)
            {
                case true:
                    controller.rumble(".");
                    controller.set_text(1, 1, "Reverse");
                    break;
                
                case false:
                    controller.rumble("...");
                    controller.set_text(1, 1, "Forward");
                    break;
            }
            pros::delay(50);
        }

        int leftY;
        int rightY;
        int leftX;
        int rightX;

        if(reversed) {
            leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
            leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
            rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        } else {
            leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
            leftX = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
            rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        }

        pros::lcd::print(0, "%f", chassis.getPose().x);
        pros::lcd::print(1, "%f", chassis.getPose().y);
        pros::lcd::print(2, "%f", chassis.getPose().theta);

        if(controller.get_digital(DIGITAL_R1)) {
            elevatorMotor1.move(127);
            elevatorMotor2.move(-127);
        } else if(controller.get_digital(DIGITAL_R2)) {
            elevatorMotor1.move(-127);
            elevatorMotor2.move(127);
        } else {
            elevatorMotor1.brake();
            elevatorMotor2.brake();
        }

        // below is experimentation with the optical sensor
        // int detectedColor = optical_sensor.get_hue();
        // std::string redOrBlue = "";
        // pros::lcd::print(2, "Optical Sensor Hue: %d", detectedColor);

        // if(detectedColor > 10 && detectedColor < 20) {
        //     pros::lcd::print(3, "Red detected");
        // }
        // else if(detectedColor > 140 && detectedColor < 160) {
        //     pros::lcd::print(3, "Blue detected");
        // }
        // else {
        //     pros::lcd::clear_line(3);
        // }

        // below is experimentation with the new potentiometers DOES NOT WORK YET
        // int pot1pos = pot1.get_value();
        // pros::lcd::print(4, "Pot1 Position: %d", pot1pos);
        // // pros::lcd::print(5, "Pot1 Calibrated Position: %d", pot1.get_value_calibrated());

        // int desiredRotations = 5;
        // if(controller.get_digital(DIGITAL_UP)) {
        //     int motorSpeed = -127;
        //     int rotations = 0;
        //     while(pot1.get_value() < desiredRotations) {
        //         if(pot1.get_value() == 4000) {
        //             rotations ++;
        //         }
        //         testMotor.move(-127);
        //         pros::lcd::print(5, "Rotations: %d", rotations);
        //     }
        //     testMotor.brake();
        // } else if(controller.get_digital(DIGITAL_DOWN)) {
        //     int rotations = 0;
        //     while(rotations < desiredRotations) {
        //         if(pot1.get_value() < desiredRotations) {
        //             rotations ++;
        //         }
        //         testMotor.move(127);
        //         pros::lcd::print(5, "Rotations: %d", rotations);
        //     }
        //     // pros::lcd::clear_line(5);
        //     testMotor.brake();
        // }

        /*  Below are some common control schemes for the robot, make sure all but one of these
            lines starting in 'chassis.' are commented out
            
            feel free to uncomment and recomment some to try different control schemes
            you can also look up different control schemes to see what they look like
        */
        
        // Tank Control
        chassis.tank(leftY, rightY);

        // Split Arcade/Drone Control
        // chassis.arcade(leftY, rightX);

        // pneumatic control
        if(controller.get_digital(DIGITAL_DOWN)) {
            sol1.set_value(LOW);
        } else if(controller.get_digital(DIGITAL_UP)) {
            sol1.set_value(HIGH);
        }

		pros::delay(20);
	}
}