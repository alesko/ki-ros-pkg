//=============================================================================
//tracker.hpp
//=============================================================================
#ifndef tracker_hpp_DEFINED
#define tracker_hpp_DEFINED
//=============================================================================


//=============================================================================
//Defines
//=============================================================================

//Allow printing errors to std output
#define TraceErrors 1

//Used station (plug)
#define STATION 1

//Time allowed for restart (ms)
#define RESTART_TIME 10000

//Max buffer size needed to receive tracker messages
#define MAX_LENGTH 255

//Default hemisphere
#define DEF_HEM_X 0
#define DEF_HEM_Y 0
#define DEF_HEM_Z 1


//Units available for XYZ
#define DIST_UNIT_MM             0
#define DIST_UNIT_CM             1
#define DIST_UNIT_M              2
#define DIST_UNIT_INCH           3
#define DEFAULT_DIST_UNIT        DIST_UNIT_MM

//Units available for angles
#define ANGLE_UNIT_RAD           0
#define ANGLE_UNIT_DEG           1
#define DEFAULT_ANGLE_UNIT       ANGLE_UNIT_RAD

//Check to see if a given int corresponds to a valid unit
#define DIST_UNIT_VALID(x) \
 (((x)==DIST_UNIT_MM)  ? (1)  :\
  ((x)==DIST_UNIT_CM)  ? (1)   :\
  ((x)==DIST_UNIT_M)   ? (1)   :\
  ((x)==DIST_UNIT_INCH)? (1):\
  (0))
#define ANGLE_UNIT_VALID(x) \
 (((x)==ANGLE_UNIT_DEG)  ? (1)  :\
  ((x)==ANGLE_UNIT_RAD)  ? (1):\
  (0))

//Printable name of unit
#define DIST_UNIT_NAME(x) \
 (((x)==DIST_UNIT_MM)  ? ("millimetres")  :\
  ((x)==DIST_UNIT_CM)  ? ("centimetres")   :\
  ((x)==DIST_UNIT_M)   ? ("metres")   :\
  ((x)==DIST_UNIT_INCH)? ("inches"):\
  ("DIST_UNIT_UNKNOWN"))
#define ANGLE_UNIT_NAME(x) \
 (((x)==ANGLE_UNIT_DEG)  ? ("degrees")  :\
  ((x)==ANGLE_UNIT_RAD)  ? ("radians"):\
  ("ANGLE_UNIT_UNKNOWN"))

//Multiplier used to convert from inches (default from polhemus) to desired
#define DIST_MULT(x) \
 (((x)==DIST_UNIT_MM)  ? (25.4)  :\
  ((x)==DIST_UNIT_CM)  ? (2.54)   :\
  ((x)==DIST_UNIT_M)   ? (0.0254)   :\
  ((x)==DIST_UNIT_INCH)? (1):\
  (10.0))

//Multiplier used to convert from deg (from polhemus) to desired
#define ANGLE_MULT(x) \
 (((x)==ANGLE_UNIT_DEG)  ? (1.0)  :\
  ((x)==ANGLE_UNIT_RAD)  ? (0.017453292519943):\
  (0.017453292519943))

//Steps for calibration
#define INTERACTIVE -1
#define STEP_0 0
#define STEP_1 1
#define STEP_2 2
#define STEP_3 3

#define TRACKER_DEF_COMNUM        1
#define TRACKER_DEFAULT_BAUD      115200
#define TRACKER_DEFAULT_STOPBITS  1
#define TRACKER_DEFAULT_DATABITS  8
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include "comport.hpp"
#include "timer.hpp"
#include "useful.h"
//=============================================================================


//=============================================================================
//Class
//=============================================================================
class tracker
{
  public:

    //Creates and opens comport with defaults, sets default values for
    //spacing, units, and hemisphere
    tracker();

    //Creates and opens comport with given com settings, sets default values
    //for spacing, units, and hemisphere
    tracker(int num, unsigned long baud, int stopbits, int databits);

    //Destructor, closes and destroys comport
    ~tracker();

    //Set system defaults, loses current origin
    void restart_tracker(void);
    void restore_system_defaults(void);
    void store_current_to_eeprom(void);



    //Functions for setting and reading origin position:

    //Three phase setup, op can be INTERACTIVE, or step (0,1,2,3)
    void setup_origin(int op);

    //Reset origin
    void clear_origin(void);

    //Manually set origin - specify 0,0 point, as well as a point on the positive x and y axes
    //This will rarely be needed
    void set_origin(double x, double y, double z, double px_x, double px_y, double px_z, double py_x, double py_y, double py_z);

    //Read origin settings
    //This will rarely be needed
    int get_origin(double* x, double* y, double* z, double* px_x, double* px_y, double* px_z, double* py_x, double* py_y, double* py_z);



    //Functions for setting and reading angle origin (boresight):

    //Single phase setup, op can be INTERACTIVE or step (0,1)
    void setup_boresight(int op);

    //Reset boresight
    void clear_boresight(void);

    //Manually set boresight - specify absolute angles which should become 0,0,0
    void set_boresight(double yaw, double pitch, double roll);

    //Set boresight to current position
    void set_boresight_to_current(void);

    //Reat boresight settings
    int get_boresight(double* yaw, double* pitch, double* roll);

    //Functions for setting and reading units, defaults are mm and rads
    void set_default_units(void);
    void set_dist_units(int dist_units);
    int get_dist_units(void);
    void set_angle_units(int angle_units);
    int get_angle_units(void);

    //Functions for setting and reading current active hemisphere
    //Default is (0,0,1), which is down if the reference is sitting upright
    void set_hemisphere(double x, double y, double z);
    void set_default_hemisphere(void);
    int get_hemisphere(double* x, double* y, double* z);

    //Function for reading pose - use NULL if value not wanted
    int get_pose(double* x, double* y, double* z, double* pitch, double* yaw, double* roll);
  //int get_multi_pose(int num, double* x, double** y, double** z, double** yaw, double** pitch, double** roll);

    //Start and stop logging - note that when start_logging is called, the next call should
    //be to stop_logging.  Most functions just return when logging, and settings cannot
    //be modified
    void start_logging(char* filename);
    void start_logging_xml(char* filename);
    void stop_logging(void);

    //Public, but don't use
    static int get_logging(void);
    static void set_logging(int val);

  private:
    //comport interface object
    comport* port;

    //Currently use units
    int dist_units_;
    int angle_units_;

    //Because boresight is not working, use these to remember current boresight
    double by, bp, br;

    //Encapsulated constructor
    void tracker_(int num, unsigned long baud, int stopbits, int databits);

    //Read boresight from tracker directly, only used in constructor
    //This would simply be the public get_boresight if boresight was
    //working on the tracker
    int get_system_boresight(double* yaw, double* pitch, double* roll);

    //Brute force function for parsing some outputs from the tracker
    int parse_stupid_string(char* buf, int num_values, double* values, int ignore_first);

    //Function to create a default log file name
    int get_default_log_filename(char* filename, int max_length);
    int get_default_xml_filename(char* filename, int max_length);

    //Used for managing logging
    pid_t child_pid_;
    static int logging_;
};
//=============================================================================


//=============================================================================
//Functions to catch signals
void catch_child(int sig_num);
void catch_interrupt(int sig_num);
//=============================================================================


//=============================================================================
#endif //ifndef tracker_hpp_DEFINED
//=============================================================================
