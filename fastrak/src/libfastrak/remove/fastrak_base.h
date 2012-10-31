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
#include "fastrak_io.h"
#include "fastrak_timer.h"
#include "useful.h"


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




enum PARITY_TYPE   { N, E, O };

typedef struct {
  char                       ttyport[MAX_NAME_LENGTH];
  int                        baud;
  enum PARITY_TYPE           parity;
  int                        fd;
  int                        databits;
  int                        stopbits;
  int                        hwf;
  int                        swf;
} polhemus_fastrak_device_t, *polhemus_fastrak_device_p;
