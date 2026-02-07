#include "autons.hpp"
#include "EZ-Template/util.hpp"
#include "antijam.hpp"
#include "dsr.hpp"
#include "intake.hpp"
#include "main.h"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

const bool RED = true;
const bool BLUE = false;

bool color;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(30.2, 0.0, 225.25);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(5.4, 0.0, 40.8);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(5.4, 0.05, 44.5, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(9.3, 0.0, 99.75);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(10, 0.0, 150);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 70);
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(9_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
  chassis.drive_imu_scaler_set(1.005);
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  chassis.pid_drive_set(72_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_drive_set(-72_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in,20_in}, fwd, DRIVE_SPEED},
                        {{24_in, 40_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set(
                      {{0_in, 0_in,0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

void measure_DSRDS_offsets(){
  DSR::measure_offsets(5);
}

// . . .
// Make your own autonomous functions here!
// . . .

void set_auton(){
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  bool not_done = true;
  while(not_done){
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
      not_done = false;
    }
    pros::delay(10);
  }
}

void right_auton(){
  //get match color
  color = intake::get_color();

  //enable anti jam
  intake::move(P_1, 127);
  antiJam::disabled = false;

  //get true position
  DSR::reset_tracking(R, B);

  //grab 5 balls
  chassis.pid_odom_set({{{97_in, 43_in}, fwd, DRIVE_SPEED}, 
  {{102_in, 55.5_in}, fwd, DRIVE_SPEED},
  {{118_in, 57_in}, fwd, DRIVE_SPEED}});

  //wait until a little before the three ball and put match load down
  chassis.pid_wait_until({90,38});
  MatchLoad.set(true);

  //wait a little more and bring matchload up
  chassis.pid_wait_until_index_started(1);  
  MatchLoad.set(false);

  //wait even more and grab the two ball
  chassis.pid_wait_quick();
  MatchLoad.set(true);
  pros::delay(300);

  //align for low goal
  chassis.pid_odom_set({{96_in, 45_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick();
  MatchLoad.set(false);

  //make sure antijam doesn't screw up
  antiJam::disabled = true;

  //turn to the low goal
  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  //outtake into lowgoal
  intake_1.move(-127);
  intake_2.move(-127);
  pros::delay(1500);
  intake::stop();

  //go to long goal/match loader
  chassis.pid_odom_set({{123_in, 24_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  //reset position
  pros::delay(100);
  DSR::reset_tracking(L, F);

  //match load
  MatchLoad.set(true);
  intake::move(P_1, 127);

  //enable antijam
  antiJam::disabled = false;

  //go to the match load tube
  chassis.pid_odom_set({{123_in, 22_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.drive_set(60,60);
  pros::delay(1000);

  //go to long goal
  chassis.pid_odom_set({{123_in, 40_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-50,-50);

  //score and stop when seeing the wrong color
  intake_2.move(127);
  intake::wait_until_color(!color, 1500);

  //disable stuff for driver
  antiJam::disabled = true;
  intake_2.move(0);

  //reset tracking again
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  intake_1.move(0);
  DSR::reset_tracking(L, F);

  //descore
  Wing.set(true);
  chassis.pid_odom_set({{117_in, 32_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_odom_set({
    {{113.5_in, 46_in}, rev, DRIVE_SPEED},
    {{113.5_in, 64_in}, rev, DRIVE_SPEED}});
  chassis.pid_wait_until_index(0);
  Wing.set(false);
  pros::delay(5000);
  chassis.pid_wait();

}

void safe_right_auton(){
  //get color for the match
  color = intake::get_color();

  //start intake and antijam
  intake::move(P_1, 127);
  antiJam::disabled = false;

  //get actual position
  DSR::reset_tracking(R, B);

  //grab the 3 ball
  chassis.pid_odom_set({{{97_in, 43_in}, fwd, DRIVE_SPEED}});
  chassis.pid_wait_until({94,34});
  MatchLoad.set(true);
  chassis.pid_wait_quick();
  pros::delay(300);

  //turn to low goal
  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  //align with low goal
  chassis.pid_odom_set({{89_in, 50_in}, fwd, DRIVE_SPEED});

  //turn off stuff that would hinder scoring
  MatchLoad.set(false);
  antiJam::disabled = true;
  chassis.pid_wait();

  //score in lowgoal
  intake_1.move(-127);
  intake_2.move(-127);
  pros::delay(1500);
  intake::stop();

  //go in front of match loader
  chassis.pid_odom_set({{121_in, 24_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick();

  //reset position
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  DSR::reset_tracking(L, F);

  //matchload
  MatchLoad.set(true);
  intake::move(P_1, 127);
  antiJam::disabled = false;
  chassis.pid_odom_set({{123_in, 22_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.drive_set(60,60);
  pros::delay(1000);

  //reset position again
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  DSR::reset_tracking(L, F);

  //score in long goal
  chassis.pid_odom_set({{123_in, 40_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-50,-50);
  intake_2.move(127);

  //wait until the wrong color with 1.5 second timeout
  intake::wait_until_color(!color, 1500);
  antiJam::disabled = true;
  intake_2.move(0);

  //reset position for descore
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  intake_1.move(0);
  DSR::reset_tracking(L, F);
  Wing.set(true);

  //desscore
  chassis.pid_odom_set({{117_in, 32_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_odom_set({
    {{114_in, 46_in}, rev, DRIVE_SPEED},
    {{114_in, 64_in}, rev, DRIVE_SPEED}});
  chassis.pid_wait_until_index(0);
  Wing.set(false);
  pros::delay(5000);
  chassis.pid_wait();

}

void left_auton(){
  color = intake::get_color();
  intake::move(P_1, 127);
  antiJam::disabled = false;
  DSR::reset_tracking(L, B);
  chassis.pid_odom_set({{{47_in, 43_in}, fwd, DRIVE_SPEED},
  {{42_in, 55.5_in}, fwd, DRIVE_SPEED},
  {{26_in, 53.5_in}, fwd, DRIVE_SPEED}});
  chassis.pid_wait_until({50,34});
  MatchLoad.set(true);
  chassis.pid_wait_until_index_started(1);
  MatchLoad.set(false);
  chassis.pid_wait_quick();
  MatchLoad.set(true);
  pros::delay(300);
  chassis.pid_odom_set({{47_in, 46_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick();
  MatchLoad.set(false);
  antiJam::disabled = true;
  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  IntakeRaise.set(false);
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in, DRIVE_SPEED);
  chassis.pid_wait();
  intake_1.move(127);
  intake_2.move(127);
  pros::delay(1500);
  IntakeRaise.set(true);
  intake::stop();
  chassis.pid_odom_set({{18_in, 24_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait_quick();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  DSR::reset_tracking(R, F);
  MatchLoad.set(true);
  intake::move(P_1, 127);
  antiJam::disabled = false;
  chassis.pid_odom_set({{20_in, 22_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.drive_set(60,60);
  pros::delay(1000);
  chassis.pid_odom_set({{20_in, 40_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-50,-50);
  intake_2.move(127);
  intake::wait_until_color(!color, 1500);
  antiJam::disabled = true;
  intake_2.move(0);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  intake_1.move(0);
  DSR::reset_tracking(R, F);
  Wing.set(true);
  chassis.pid_odom_set({{27_in, 32_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_odom_set({
    {{32_in, 46_in}, fwd, DRIVE_SPEED},
    {{30_in, 60_in}, fwd, DRIVE_SPEED}});
  chassis.pid_wait_until_index(0);
  Wing.set(false);
  MatchLoad.set(false);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(30_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(0_in, DRIVE_SPEED);
  pros::delay(5000);
  chassis.pid_wait();
}

void safe_left_auton(){
  color = intake::get_color();
  intake::move(P_1, 127);
  antiJam::disabled = false;
  DSR::reset_tracking(L, B);
  chassis.pid_odom_set({{{47_in, 43_in}, fwd, DRIVE_SPEED}});
  chassis.pid_wait_until({50,34});
  pros::delay(300);
  chassis.pid_odom_set({{47_in, 46_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick();
  antiJam::disabled = true;
  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  IntakeRaise.set(false);
  chassis.pid_wait();
  chassis.pid_drive_set(-4_in, DRIVE_SPEED);
  chassis.pid_wait();
  intake_1.move(127);
  intake_2.move(127);
  pros::delay(1500);
  IntakeRaise.set(true);
  intake::stop();
  chassis.pid_odom_set({{18_in, 24_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait_quick();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  DSR::reset_tracking(R, F);
  MatchLoad.set(true);
  intake::move(P_1, 127);
  antiJam::disabled = false;
  chassis.pid_odom_set({{20_in, 22_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.drive_set(60,60);
  pros::delay(1000);
  chassis.pid_odom_set({{20_in, 40_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-50,-50);
  intake_2.move(127);
  intake::wait_until_color(!color, 1500);
  antiJam::disabled = true;
  intake_2.move(0);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  intake_1.move(0);
  DSR::reset_tracking(R, F);
  Wing.set(true);
  chassis.pid_odom_set({{27_in, 32_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_odom_set({
    {{32_in, 46_in}, fwd, DRIVE_SPEED},
    {{30_in, 60_in}, fwd, DRIVE_SPEED}});
  chassis.pid_wait_until_index(0);
  Wing.set(false);
  MatchLoad.set(false);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(30_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(0_in, DRIVE_SPEED);
  pros::delay(5000);
  chassis.pid_wait();
}

void skills_auton(){
  intake_1.move(127);
  chassis.pid_drive_set(15_in, DRIVE_SPEED);
  //wait until stop time, so just make sure you like how the clear went
  pros::delay(50000);
}

void skills_but_better(){
  antiJam::disabled = false;
  intake::move(P_1, 127);
  intake_1.move(0);
  chassis.odom_theta_set(90_deg);
  chassis.pid_odom_set({{{20_in, 10_in}, fwd, DRIVE_SPEED},
  {{36_in, 0_in}, fwd, DRIVE_SPEED},
  {{39_in, -3_in}, fwd, DRIVE_SPEED}});
  chassis.pid_wait_until(1);
  MatchLoad.set(true);
  intake_1.move(127);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.drive_set(60,60);
  pros::delay(2000);
  chassis.pid_drive_set(-5_in, DRIVE_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  DSR::reset_tracking(L, F);
  chassis.pid_odom_set({{{136_in, 38_in}, rev, 3 * DRIVE_SPEED / 4},
  {{136_in, 108_in}, rev, DRIVE_SPEED},
  {{122_in, 120_in}, rev, 3 * DRIVE_SPEED / 4}});
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  DSR::reset_tracking(R, F);
  chassis.pid_odom_set({{122.5_in,107_in}, rev, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.drive_set(-50,-50);
  pros::delay(300);
  intake_2.move(127);
  pros::delay(2000);
  intake_2.move(0);
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  DSR::reset_tracking(R, F);
  chassis.pid_odom_set({{123_in, 125_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();
  chassis.drive_set(60,60);
  pros::delay(2000);
  chassis.pid_odom_set({{123.5_in,104_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();
  chassis.drive_set(-50,-50);
  pros::delay(300);
  intake_2.move(127);
  pros::delay(2000);
  intake_2.move(0);
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  DSR::reset_tracking(R, F);
  MatchLoad.set(false);
  chassis.pid_odom_set({{121_in, 124_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set({91_in, 131_in}, fwd, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_odom_set({{91_in, 131_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.drive_set(50,50);
  pros::delay(500);
  chassis.pid_drive_set(48_in, 80);
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  DSR::reset_tracking(L, F);
  chassis.pid_odom_set({{20_in, 115_in}, rev, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  DSR::reset_tracking(L, F);
  chassis.pid_odom_set({{20_in, 107_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick();
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_odom_set({{20_in, 104_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();
  chassis.drive_set(-50,-50);
  pros::delay(300);
  intake_2.move(127);
  pros::delay(2000);
  intake_2.move(0);
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  DSR::reset_tracking(L, F);
  MatchLoad.set(true);
  chassis.drive_set(60,60);
  pros::delay(2000);
  chassis.pid_odom_set({{{10_in, 106_in}, rev, 3 * DRIVE_SPEED / 4},
  {{10_in, 38_in}, rev, DRIVE_SPEED},
  {{18_in, 24_in}, rev, 3 * DRIVE_SPEED / 4}});
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  DSR::reset_tracking(R, F);
  chassis.pid_odom_set({{21_in, 38_in}, rev, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.drive_set(-50,-50);
  pros::delay(300);
  intake_2.move(127);
  pros::delay(2000);
  intake_2.move(0);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  DSR::reset_tracking(R, F);
  chassis.pid_odom_set({{22_in, 19_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();
  chassis.drive_set(60,60);
  pros::delay(2000);
  chassis.pid_odom_set({{21_in, 40_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();
  chassis.drive_set(-50,-50);
  pros::delay(300);
  intake_2.move(127);
  pros::delay(2000);
  MatchLoad.set(false);
  intake_1.move(0);
  intake_2.move(0);
  antiJam::disabled = true;
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  DSR::reset_tracking(R, F);
  chassis.pid_odom_set({{{20_in, 30_in}, fwd, DRIVE_SPEED},
  {{24_in, 18_in}, fwd, DRIVE_SPEED},
  {{48_in,8_in}, fwd, 3 * DRIVE_SPEED / 4}});
  chassis.pid_wait();
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.drive_set(50,50);
  pros::delay(500);
  chassis.pid_drive_set(28_in, 80);
  intake_1.move(127);
  pros::delay(5000);
}

void mover(){
  chassis.pid_drive_set(5_in, DRIVE_SPEED);
  chassis.pid_wait();
}

void SAWP(){
  chassis.odom_theta_set(-90_deg);
  pros::delay(100);
  antiJam::disabled = false;
  intake::move(P_1, 127);
  DSR::reset_tracking(B, L);
  chassis.pid_drive_set(10_in, DRIVE_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{123_in, 30_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  MatchLoad.set(true);
  pros::delay(100);
  DSR::reset_tracking(L, F);
  chassis.pid_odom_set({{122_in, 23_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  DSR::reset_tracking(L, F);
  chassis.drive_set(60,60);
  pros::delay(1200);
  chassis.pid_odom_set({{123_in, 40_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-50,-50);
  intake_2.move(127);
  intake::wait_until_color(!color, 2000);
  intake_2.move(0);
  MatchLoad.set(false);
  chassis.pid_turn_set(180_deg,TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  DSR::reset_tracking(L,F);
  chassis.pid_odom_set({{{116_in, 50_in}, fwd, DRIVE_SPEED},
  {{62_in, 50_in}, fwd, DRIVE_SPEED}});
  chassis.pid_wait_until_index(0);
  MatchLoad.set(true);
  pros::delay(600);
  MatchLoad.set(false);
  chassis.pid_wait_quick();
  MatchLoad.set(true);
  chassis.pid_turn_set(-135_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  IntakeRaise.set(false);
  chassis.pid_drive_set(-16_in, DRIVE_SPEED);
  chassis.pid_wait_quick();
  intake_1.move(127);
  intake_2.move(127);
  pros::delay(1500);
  IntakeRaise.set(true);
  intake::stop();
  intake_1.move(127);
  chassis.pid_odom_set({{{40_in, 28_in}, fwd, DRIVE_SPEED},{{30_in, 32_in}, fwd, DRIVE_SPEED}});
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  pros::delay(800);
  chassis.pid_wait_quick();
  DSR::reset_tracking(R, F);
  MatchLoad.set(true);
  chassis.pid_odom_set({{20_in, 23_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  pros::delay(100);
  DSR::reset_tracking(R, F);
  chassis.drive_set(60,60);
  pros::delay(1200);
  chassis.pid_odom_set({{19_in, 40_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-50,-50);
  intake_2.move(127);
  intake::wait_until_color(!color, 1500);
  antiJam::disabled = true;
  intake_2.move(0);
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(100);
  intake_1.move(0);
  DSR::reset_tracking(R, F);
  Wing.set(true);
  chassis.pid_odom_set({{27_in, 32_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  chassis.pid_odom_set({
    {{32_in, 46_in}, fwd, DRIVE_SPEED},
    {{30_in, 60_in}, fwd, DRIVE_SPEED}});
  chassis.pid_wait_until_index(0);
  Wing.set(false);
  MatchLoad.set(false);
  chassis.pid_wait_quick();
  chassis.pid_turn_set(30_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(0_in, DRIVE_SPEED);
  pros::delay(5000);
  chassis.pid_wait();
  antiJam::disabled = true;
}

void super_safe_right(){
  //get match color
  color = intake::get_color();

  //intake start
  intake_1.move(127);

  //first three balls
  chassis.pid_odom_set({{5_in, 33_in}, fwd, DRIVE_SPEED});

  //timing for matchloader
  pros::delay(600);
  MatchLoad.set(true);
  chassis.pid_wait_quick_chain();

  //go back a bit because went too far
  chassis.pid_drive_set(-3_in, DRIVE_SPEED);
  chassis.pid_wait_quick();

  //align to low goal
  chassis.pid_turn_set({-1_in, 40_in}, fwd, DRIVE_SPEED);
  chassis.pid_wait();
  MatchLoad.set(false);

  //go to low goal
  chassis.pid_odom_set({{-1_in, 40_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait_quick();

  //score in low goal
  intake_1.move(-127);
  intake_2.move(-127);
  pros::delay(1500);

  //go a bit further to put one more ball in low goal
  chassis.drive_set(40,40);
  pros::delay(200);

  //stop intake
  intake::stop();

  //go to general long goal/matchload area
  chassis.pid_odom_set({{30_in, 14_in}, rev, DRIVE_SPEED});
  chassis.pid_wait();

  //align to matchloader
  chassis.pid_turn_set({29_in, -4_in}, fwd, TURN_SPEED);
  MatchLoad.set(true);
  chassis.pid_wait();

  //go to and intake matchloader
  intake_1.move(127);
  chassis.pid_odom_set({{29_in, -1_in}, fwd, DRIVE_SPEED / 2});
  chassis.pid_wait_quick_chain();

  //constant presssure to make it work DO NOT REMOVE
  chassis.drive_set(60,60);
  pros::delay(1000);

  //go to long goal
  chassis.pid_odom_set({{29_in, 28_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();

  //constant pressure to make sure i am aligned
  chassis.drive_set(-50,-50);
  intake_2.move(127);

  //wait until wrong color
  intake::wait_until_color(!color, 2500);
  intake_2.move(0);
  intake_1.move(0);
  MatchLoad.set(false);

  //descore doesnt really matter
  chassis.pid_odom_set({{22_in, 20_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait();
  Wing.set(true);

  //descore actual alignment, please change if necessary
  chassis.pid_odom_set({{{18_in, 26_in}, rev, DRIVE_SPEED}, {{19_in, 44_in}, rev, DRIVE_SPEED}});
  
  //the rest dont matter
  chassis.pid_wait_until_index(0);
  Wing.set(false);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(170_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(0_in , DRIVE_SPEED);
  pros::delay(10000);
}

void super_safe_left(){

  //same thing for left but with slightly different numbers and a different turn to mid goal + descore foreward instead of backward
  color = intake::get_color();
  intake_1.move(127);
  chassis.pid_odom_set({{-5_in, 33_in}, fwd, DRIVE_SPEED});
  pros::delay(600);
  MatchLoad.set(true);
  chassis.pid_wait_quick_chain();
  chassis.pid_drive_set(-3_in, DRIVE_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_turn_set({1.5_in, 42.5_in}, rev, DRIVE_SPEED);
  chassis.pid_wait();
  IntakeRaise.set(false);
  MatchLoad.set(false);
  chassis.pid_odom_set({{1.5_in, 42.5_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick();
  intake_1.move(127);
  intake_2.move(127);
  pros::delay(1500);
  intake::stop();
  chassis.pid_odom_set({{-30_in, 14_in},fwd, DRIVE_SPEED});
  chassis.pid_wait();
  IntakeRaise.set(true);
  chassis.pid_turn_set({-33_in, -4_in}, fwd, TURN_SPEED);
  MatchLoad.set(true);
  chassis.pid_wait();
  intake_1.move(127);
  chassis.pid_odom_set({{-33_in, -1_in}, fwd, DRIVE_SPEED / 2});
  chassis.pid_wait_quick_chain();
  chassis.drive_set(60,60);
  pros::delay(1000);
  chassis.pid_odom_set({{-33_in, 28_in}, rev, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();
  chassis.drive_set(-50,-50);
  intake_2.move(127);
  intake::wait_until_color(!color, 2500);
  intake_2.move(0);
  intake_2.move(0);
  MatchLoad.set(false);
  chassis.pid_odom_set({{-33_in, 20_in}, fwd, DRIVE_SPEED});
  chassis.pid_wait_quick();
  Wing.set(true);
  chassis.pid_turn_set({-24_in, 26_in}, fwd, TURN_SPEED);
  chassis.pid_wait_quick_chain();
  chassis.pid_odom_set({{{-24_in, 26_in}, fwd, DRIVE_SPEED}, {{-25_in, 40_in}, fwd, DRIVE_SPEED}});
  chassis.pid_wait_until_index(0);
  Wing.set(false);
  chassis.pid_wait_quick_chain();
  chassis.pid_turn_set(45_deg, TURN_SPEED / 2);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(0_in , DRIVE_SPEED);
  pros::delay(10000);
}