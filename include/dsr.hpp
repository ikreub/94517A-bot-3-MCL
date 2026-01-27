#pragma once

#include <string>
#include <vector>
#include "../include/EZ-Template/api.hpp"
#include "pros/distance.hpp"
#include "main.h"

/*!
* \enum Dir
* \brief The direction of the sensor, used for determining how to measure offsets and reset odom tracking
*/
enum Dir{
    f = 1,
    r = 2,
    b = 3,
    l = 4,
    left = l,
    right = r,
    front = f,
    back = b,
    L = l,
    R = r,
    F = f,
    B = b,
    Left = l,
    Right = r,
    Front = f,
    Back = b,
    LEFT = l,
    RIGHT = r,
    FRONT = f,
    BACK = b
};

/*!
* \class DSRDS
* \brief A class representing a distance sensor used in DSR. 
* 
* It has the ability to:
*
*   -read in inches
*
*   -set and get offsets
*
*   -measure offsets based on multiple readings
*   
*/
class DSRDS{
    public:

    /*!
    * \brief constructor for DSRDS
    * \param port the port the sensor is plugged into
    * \param direction the direction the sensor is facing, used for determining how to measure offsets and reset odom tracking
    * \param x_offset the x offset of the sensor in inches, used for odom resets, calculated by offset measurements
    * \param y_offset the y offset of the sensor in inches, used for odom resets, calculated by offset measurements
    */
    DSRDS(int port, Dir direction, double offset = 0);

    /*!
    * \brief helper function for read_in
    */
    double read();

    /*!
    * \brief read from the sensor in inches
    */
    double read_in();

    /*!
    * \brief set x offset
    * \param x the x offset in inches
    */
    void set_x_offset(double x);

    /*!
    * \brief set y offset
    * \param y the y offset in inches
    */
    void set_y_offset(double y);

    /*!
    * \brief set both offsets at once
    * \param x the x offset in inches
    * \param y the y offset in inches
    */
    void set_offsets(double x, double y);

    /*!
    * \brief set the direction of the sensor
    * \param direction the direction of the sensor
    */
    void set_dir(Dir direction);

    /*!
    * \brief set the offset in the direction of the sensor
    * \param offset the direction of the sensor
    */
    void set_dir_offset(double offset);

    /*!
    * \brief get the x offset
    * \return the x offset in inches
    */
    double get_x_offset();

    /*!
    * \brief get the y offset
    * \return the y offset in inches
    */
    double get_y_offset();

    /*!
    * \brief get the offsets as a pose
    * \return the offsets as a pose
    */
    ez::pose get_offsets();

    /*!
    * \brief get the direction of the sensor
    * \return the direction of the sensor
    */
    Dir get_dir();

    /*!
    * \brief get the offset in the direction of the sensor
    * \return the offset in the direction of the sensor
    */
    double get_dir_offset();

    /*!
    * \brief get the direction of the sensor as a string (for debugging purposes)
    * \return the direction of the sensor as a string
    */
    std::string get_dir_string();

    /*!
    * \brief measure the offsets of the sensor
    * \param read45 the reading from the sensor at 45 degrees
    * \param read30 the reading from the sensor at 30 degrees
    * \param read0 the reading from the sensor at 0 degrees
    */
    void measure_offsets(double read45, double read30, double read0);

    /*!
    * \private memvers of the class
    */
    private:

    /*!
    * \brief the offsets of the sensor in inches, used for odom resets, calculated by offset measurements
    */
    double x_offset;

    /*!
    * \brief the y offsets of the sensor in inches, used for odom resets, calculated by offset measurements
    */
    double y_offset;

    /*!
    * \brief the offset in thee direction the sensor is facing.
    */
    double dir_offset;

    /*!
    * \brief the direction of the sensor, used for determining how to measure offsets and reset odom tracking
    */
    Dir dir;

    /*!
    * \brief the direction of the sensor as a string, used for debugging purposes
    */
    std::string dir_string;

    /*!
    * \brief the distance sensor object from the PROS API, used to read values from the sensor
    */
    pros::Distance sensor;

    /*!
    * \brief a helper function to convert the direction enum to a string for debugging purposes
    * \param direction the direction of the sensor
    * \return the direction as a string
    */
    std::string dir_to_string(Dir direction){
        switch(int(direction)){
            case int(l):
                return "L";
            case int(r):
                return "R";
            case int(f):
                return "F";
            case int(b):
                return "B";
            default:
                return "";
        }
    }
};

/*! \namespace DSR
 *  \brief All functions in DSR that need to be accessible
 *  
 *  It has the ability to:
 *
 *      -Set all sensors
 *
 *      -Add individual sensors
 *
 *      -Reset odom tracking
 *
 *      -Measure sensor offsets
 *      
 */
namespace DSR{

    /*!
    * \brief sets the list of sensors
    *
    * \param sensors the list of referenced sensors
    */
    void set_sensors(std::vector<DSRDS&> sensors);

    /*!
    * \brief add a sensor to the list
    *
    * \param sensor the referenced sensor
    */
    void add_sensor(DSRDS& sensor);

    /*!
    * \brief used in autonomous to reset the tracking values based on specified distance sensor readings.
    * \param sensorX_dir the direction of the sensor used for x tracking
    * \param sensorY_dir the direction of the sensor used for y tracking
    * \param sensorX_specified the specific sensor used for x tracking (defaults to the first one mentioned)
    * \param sensorY_specified the specific sensor used for y tracking (defaults to the first one mentioned)
    */
    void reset_tracking(Dir sensorX_dir, Dir sensorY_dir,int sensorX_specified = 1, int sensorY_specified = 1 );
    
    /*!
    * \brief measure offsets for all used sensors.
    * \param iterations the number of times to read each sensor for better accuracy
    *
    * Put the bot against a wall facing the wall and run the auton. It will back up and tuen a bunch of times to measure the offsets of each sensor.
    */
    void measure_offsets(int iterations);

    /*!
    * \brief The list of sensors
    */
    inline std::vector<DSRDS> sensors = {};
}