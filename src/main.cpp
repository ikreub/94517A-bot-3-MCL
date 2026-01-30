#include "main.h"
#include "EZ-Template/sdcard.hpp"
#include "EZ-Template/util.hpp"
#include "antiJam.hpp"
#include "antijam.hpp"
#include "autons.hpp"
#include "dsr.hpp"
#include "pros/misc.h"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

//initialize DSR sensors

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-18, -19, 20},     // Left Chassis Ports (negative port will reverse it!)
    {1, -2, 3},  // Right Chassis Ports (negative port will reverse it!)

    11,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
ez::tracking_wheel horiz_tracker(10, 2, 0.03);  // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker(-15, 2.75, 1.88);   // This tracking wheel is parallel to the drive wheels
DSRDS front_sensor(17, Front, 9.83);
DSRDS Left_sensor(16, Left, 2.87);
DSRDS back_sensor(13, Back, 1.1);
DSRDS right_sensor(7, Right, 3.16);
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  chassis.odom_tracker_right_set(&vert_tracker);

  DSR::add_sensor(front_sensor);
  DSR::add_sensor(Left_sensor);
  DSR::add_sensor(back_sensor);
  DSR::add_sensor(right_sensor);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();
  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // Initialize MCL sensors

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"move \n NOTE: all autons can be started with A and Left \n this INCLUDES in driver\nthere is no overide so be careful not to start auton in driver \n this is also how you would do a skills macro in auton", mover},
      {"set auton", set_auton},
      {"Right side", right_auton},
      {"safe  right side", safe_right_auton},
      {"Left side", left_auton},
      {"Safe left side \n WARNING: i dont think it will work on middle goal", safe_left_auton},
      {"Solo auto win point \n WARNING: i didn't get much testing, \n test it on the skills or practice field first", SAWP},
      {"Skills", skills_but_better},
      {"Measure Offsets\n\nThis will turn the robot a bunch of times and calculate your offsets for your tracking wheels.", measure_offsets},
      {"Measure DSRDS offsets", measure_DSRDS_offsets},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

pros::Task aniJammer(antiJam::Task);

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
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
  // . . .
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
  colorStop.set_led_pwm(100);
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position, you can start at a specific position with this
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  /*
  Odometry and Pure Pursuit are not magic

  It is possible to get perfectly consistent results without tracking wheels,
  but it is also possible to have extremely inconsistent results without tracking wheels.
  When you don't use tracking wheels, you need to:
   - avoid wheel slip
   - avoid wheelies
   - avoid throwing momentum around (super harsh turns, like in the example below)
  You can do cool curved motions, but you have to give your robot the best chance
  to be consistent
  */

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
  colorStop.set_led_pwm(0);
}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */
void ez_screen_task() {
  while (true) {
    // Only run this when not connected to a competition switch
    if (!pros::competition::is_connected()) {
      // Blank page for odom debugging
      if (chassis.odom_enabled() && !chassis.pid_tuner_enabled()) {
        // If we're on the first blank page...
        if (ez::as::page_blank_is_on(0)) {
          // Display X, Y, and Theta
          ez::screen_print("x: " + util::to_string_with_precision(chassis.odom_x_get()) +
                               "\ny: " + util::to_string_with_precision(chassis.odom_y_get()) +
                               "\na: " + util::to_string_with_precision(chassis.odom_theta_get()),
                           1);  // Don't override the top Page line

          // Display all trackers that are being used
          screen_print_tracker(chassis.odom_tracker_left, "l", 4);
          screen_print_tracker(chassis.odom_tracker_right, "r", 5);
          screen_print_tracker(chassis.odom_tracker_back, "b", 6);
          screen_print_tracker(chassis.odom_tracker_front, "f", 7);
        }
        if(ez::as::page_blank_is_on(1)){
            for(unsigned int i = 0; i < DSR::sensors.size(); i++){
            ez::screen_print(DSR::sensors[i].get_dir_string() + " sensor: " + util::to_string_with_precision(DSR::sensors[i].read_in()), 1 + int(i));
          }
        }
        if(ez::as::page_blank_is_on(2)){
          for(unsigned int i = 0; i < DSR::sensors.size(); i++){
            ez::screen_print(util::to_string_with_precision(antiJam::disable), 5);
            ez::screen_print(util::to_string_with_precision(intake_1.get_efficiency()), 6);
            ez::screen_print(DSR::sensors[i].get_dir_string() +  "offset: " + util::to_string_with_precision(DSR::sensors[int(i)].get_dir_offset()), 1 + int(i));
          }
        }
        if(ez::as::page_blank_is_on(3)){
          ez::screen_print(util::to_string_with_precision(colorStop.get_proximity()));
        }
      }
    }

    // Remove all blank pages when connected to a comp switch
    else {
      if (ez::as::page_blank_amount() > 0)
        ez::as::page_blank_remove_all();
    }

    pros::delay(ez::util::DELAY_TIME);
  }
}
pros::Task ezScreenTask(ez_screen_task);

/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
bool done = false;
Dir X;
Dir Y;
void ez_template_extras() {
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    

    if(master.get_digital(DIGITAL_A) &&  master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
      pros::delay(500);
      done = false;
      while(!done){
        if(master.get_digital_new_press(DIGITAL_UP)){
          X = F;
          done = true;
        }else if(master.get_digital_new_press(DIGITAL_LEFT)){
          X = L;
          done = true;
        }else if(master.get_digital_new_press(DIGITAL_DOWN)){
          X = B;
          done = true;
        }else if(master.get_digital_new_press(DIGITAL_RIGHT)){
          X = R;
          done = true;
        }
      }
      pros::delay(500);
      done = false;
      while(!done){
        if(master.get_digital_new_press(DIGITAL_UP)){
          Y = F;
          done = true;
        }else if(master.get_digital_new_press(DIGITAL_LEFT)){
          Y = L;
          done = true;
        }else if(master.get_digital_new_press(DIGITAL_DOWN)){
          Y = B;
          done = true;
        }else if(master.get_digital_new_press(DIGITAL_RIGHT)){
          Y = R;
          done = true;
        }
      }
      DSR::reset_tracking(X, Y);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
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
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  IntakeRaise.set(true);
  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();
    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_A) && master.get_digital(DIGITAL_LEFT)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .
    intake::opcontrol();

    IntakeRaise.button_toggle(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN));
    (IntakeRaise.get() == true) ? MatchLoad.button_toggle(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) : MatchLoad.set(false);
    DoublePark.button_toggle(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT));
    (IntakeRaise.get() == true) ? Wing.button_toggle(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) : Wing.set(false);


    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}
