//=============================================================================
//tracker.cpp
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include "tracker.hpp"
//=============================================================================


//=============================================================================
//Static data member
//=============================================================================
int tracker::logging_;
//=============================================================================


//=============================================================================
//Functions
//=============================================================================
tracker::tracker(int num, unsigned long baud, int stopbits, int databits)
{
  tracker_(num, baud, stopbits, databits);
}
//=============================================================================
tracker::tracker(void)
{
  tracker_(TRACKER_DEF_COMNUM,
           TRACKER_DEFAULT_BAUD,
           TRACKER_DEFAULT_STOPBITS,
           TRACKER_DEFAULT_DATABITS);
}
//=============================================================================
void
tracker::tracker_(int num, unsigned long baud, int stopbits, int databits)
{
  //This is an encapsulated constructor

  //Creates and opens comport with given settings, sets defaults
  //for units and hemisphere
  double yaw, pitch, roll;
  port = new comport;

  if (port->open_comport(num, baud, stopbits, databits) == -1)
  {
    #if TraceErrors
    printf("tracker: couldn't open comport\n");
    return;
    #endif
  }

  //Always use inches from tracker since this is the system default
  //Conversion is done upon reading
  port->send_comport("U");

  //On startup, read system stored boresight and set our boresight to this
  set_angle_units(ANGLE_UNIT_DEG);
  get_system_boresight(&yaw, &pitch, &roll); //always in degrees
  set_boresight(yaw, pitch, roll);

  //Set defaults
  set_default_units();
  set_default_hemisphere();


  //FOR LOGGING:

  //Logging flag
  set_logging(0);

  //Child process stuff
  signal(SIGCHLD, catch_child);
  signal(SIGALRM, catch_interrupt);
  child_pid_ = -1;
}
//=============================================================================
tracker::~tracker(void)
{
  //Closes and destroys comport, destroys self
  port->close_comport();
  delete port;
}
//=============================================================================
void
tracker::restart_tracker(void)
{
  if (get_logging())
    return;

  timer t;
  char buf[MAX_LENGTH];
  sprintf(buf, "%c", 0x19);
  port->send_comport(buf);
  t.timer_sleep(RESTART_TIME);//alow time seconds for system restart
}
//=============================================================================
void
tracker::restore_system_defaults(void)
{
  if (get_logging())
    return;

  port->send_comport("W");
  set_boresight(0.0, 0.0, 0.0); //Need to do this manually since boresight not working
}
//=============================================================================
void
tracker::store_current_to_eeprom(void)
{
  if (get_logging())
    return;

  char buf[MAX_LENGTH];
  sprintf(buf, "%c", 0x0b);
  port->send_comport(buf);
}
//=============================================================================
void
tracker::setup_origin(int op)
{
  if (get_logging())
    return;

  //Three step calibration procedure
  char buf[MAX_LENGTH];
  switch(op)
  {
    case INTERACTIVE:
      clear_origin();
      port->send_comport("A0\r");
      port->recv_comport(buf, MAX_LENGTH);
      printf("Place probe at desired xyz origin, then hit enter...\n");
      getchar();
      port->send_comport("P");
      port->recv_comport(buf, MAX_LENGTH);
      printf("Place probe 20-40cm along the positive X axis, then hit enter...\n");
      getchar();
      port->send_comport("P");
      port->recv_comport(buf, MAX_LENGTH);
      printf("Place probe 20-40cm along the positive Y axis, then hit enter...\n");
      getchar();
      port->send_comport("P");
      port->recv_comport(buf, MAX_LENGTH);
      printf("Done!\n");
      break;
    case STEP_0:
      clear_origin();
      port->send_comport("A0\r");
      port->recv_comport(buf, MAX_LENGTH);
      break;
    case STEP_1:  //Place probe at origin
    case STEP_2:  //Place probe on X
    case STEP_3:  //Place probe on Y
      port->send_comport("P");
      port->recv_comport(buf, MAX_LENGTH);
      break;
    default:
      #if TraceErrors
      printf("tracker: invalid op given to setup_xyz_origin\n");
      #endif
  }
}
//=============================================================================
void
tracker::clear_origin(void)
{
  if (get_logging())
    return;

  char buf[MAX_LENGTH];
  sprintf(buf, "R%d\r", STATION);
  port->send_comport(buf);
}
//=============================================================================
void
tracker::set_origin(double x, double y, double z, double px_x, double px_y, double px_z, double py_x, double py_y, double py_z)
{
  if (get_logging())
    return;

  char buf[MAX_LENGTH];

  sprintf(buf, "A%d,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf,%.2lf\r", STATION, x, y, z, px_x, px_y, px_z, py_x, py_y, py_z);
  port->send_comport(buf);

}
//=============================================================================
int
tracker::get_origin(double* x, double* y, double* z, double* px_x, double* px_y, double* px_z, double* py_x, double* py_y, double* py_z)
{
  if (get_logging())
    return -1;

  double tx, ty, tz, tpx_x, tpx_y, tpx_z, tpy_x, tpy_y, tpy_z;

  //Read hemisphere of operation
  char buf[MAX_LENGTH];
  sprintf(buf, "A%d\r", STATION);
  port->send_comport(buf);
  port->recv_comport(buf, MAX_LENGTH);
  if (sscanf(buf, "%*s %lf %lf %lf", &tx, &ty, &tz, &tpx_x, &tpx_y, &tpx_z, &tpy_x, &tpy_y, &tpy_z) != 9)
    return -1;

  *x = tx;
  *y = ty;
  *z = tz;
  *px_x = tpx_x;
  *px_y = tpx_y;
  *px_z = tpx_z;
  *py_x = tpy_x;
  *py_y = tpy_y;
  *py_z = tpy_z;
  return 1;
}
//=============================================================================
void
tracker::setup_boresight(int op)
{
  if (get_logging())
    return;

  //One step calibration procedure
  switch(op)
  {
    case INTERACTIVE:
      clear_boresight();
      printf("Place probe at desired origin orientation, then hit enter...");
      getchar();
      set_boresight_to_current();
      printf("Done!\n");
      break;
    case STEP_0:
      clear_boresight();
      break;
    case STEP_1:  //Place probe at origin
      set_boresight_to_current();
      break;
    default:
      #if TraceErrors
      printf("tracker: invalid op given to setup_angles_origin\n");
      #endif
  }
}
//=============================================================================
void
tracker::clear_boresight(void)
{
  if (get_logging())
    return;

  //char buf[MAX_LENGTH];
  //sprintf(buf, "b%d\r", STATION);
  //port->send_comport(buf);

  //Boresight not working, do it manually
  set_boresight(0.0, 0.0, 0.0);
}
//=============================================================================
void
tracker::set_boresight(double yaw, double pitch, double roll)
{
  if (get_logging())
    return;

  //Wouldn't need to bother with by,bp,br if boresight was working on tracker
  char buf[MAX_LENGTH];

  if (get_angle_units() == ANGLE_UNIT_RAD)
  {
    by = DEG(yaw);
    bp = DEG(pitch);
    br = DEG(roll);
  }
  else
  {
    by = yaw;
    bp = pitch;
    br = roll;
  }

  sprintf(buf, "G%d,%.2lf,%.2lf,%.2lf\r", STATION, by, bp, br);
  port->send_comport(buf);

  /*
  by = NORM_DEG(by);
  bp = NORM_DEG(bp);
  br = NORM_DEG(br);
  */
}
//=============================================================================
void
tracker::set_boresight_to_current(void)
{
  if (get_logging())
    return;

  //char buf[MAX_LENGTH];
  //sprintf(buf, "B%d\r", STATION);
  //port->send_comport(buf);

  //Boresight not working, do it manually
  double yaw, pitch, roll;
  get_pose(NULL, NULL, NULL, &yaw, &pitch, &roll);
  set_boresight(yaw, pitch, roll);
}
//=============================================================================
int
tracker::get_boresight(double* yaw, double* pitch, double* roll)
{
  //This would contain what is now in get_system_boresight() if boresight
  //was working on tracker

  if (get_angle_units() == ANGLE_UNIT_RAD)
  {
    *yaw = RAD(by);
    *pitch = RAD(bp);
    *roll = RAD(br);
  }
  else
  {
    *yaw = by;
    *pitch = bp;
    *roll = br;
  }

  return 1;
}
//=============================================================================
int
tracker::get_system_boresight(double* yaw, double* pitch, double* roll)
{
  if (get_logging())
    return -1;

  //Read system boresight, only used by constructor
  //This would be get_boresight(), if boresight was working on tracker
  //double y,p,r;
  double values[3];
  char buf[MAX_LENGTH];

  sprintf(buf, "G%d\r", STATION);
  port->send_comport(buf);
  port->recv_comport(buf, MAX_LENGTH);

  parse_stupid_string(buf, 3, values, 3);

  *yaw = values[0];
  *pitch = values[1];
  *roll = values[2];

  return 1;
}
//=============================================================================
int
tracker::parse_stupid_string(char* buf, int num_values, double* values, int ignore_first)
{
  char item[MAX_LENGTH];
  int i,j;
  int start[num_values];
  int end[num_values];

  //Output from tracker is NOT formatted well!!!
  //Spaces separate the numbers if they are positive, but negative numbers
  //can cause two numbers to be side by side
  //This is a sloppy brute force parsing algorithm for these situations

  //Find start and end of all numbers (numbers consist of digits, -, or .)
  //'-' can only be at the start of a number

  for (i=0;i<num_values;i++)
  {
    //Ignore first chars
    if (i==0)
      start[i]=ignore_first;
    else
      start[i]=end[i-1]+1;
    
    while(((buf[start[i]] < '0') || (buf[start[i]] > '9')) && (buf[start[i]] != '-') && (buf[start[i]] != '.'))
      start[i]++;

    end[i]=start[i]+1;
    while(((buf[end[i]] >= '0') && (buf[end[i]] <= '9')) || (buf[end[i]] == '.'))
      end[i]++;
    end[i]--;

    for (j=start[i];j<=end[i];j++)
      item[j-start[i]] = buf[j];

    item[j-start[i]] = '\0';

    if (sscanf(item, "%lf", &(values[i])) != 1)
    {
        #if TraceErrors
        printf("tracker: error parsing string\n");
        #endif
        return -1;
    }
  }

  /*
  printf("buf<%s> became: (", buf);
  for (i=0;i<num_values;i++) printf("%lf, ", values[i]);
  printf(")\n");
  */

  return 1;
}
//=============================================================================
void
tracker::set_default_units(void)
{
  if (get_logging())
    return;

  set_dist_units(DEFAULT_DIST_UNIT);
  set_angle_units(DEFAULT_ANGLE_UNIT);
}
//=============================================================================
void
tracker::set_dist_units(int dist_units)
{
  if (get_logging())
    return;

  //if valid assign, otherwise print error
  if (DIST_UNIT_VALID(dist_units))
    dist_units_ = dist_units;
  else
  {
    #if TraceErrors
    printf("tracker: invalid distance unit, current unit not changed\n");
    #endif
  }
}
//=============================================================================
int
tracker::get_dist_units(void)
{
  return dist_units_;
}
//=============================================================================
void
tracker::set_angle_units(int angle_units)
{
  if (get_logging())
    return;

  if (ANGLE_UNIT_VALID(angle_units))
    angle_units_ = angle_units;
  else
  {
    #if TraceErrors
    printf("tracker: invalid distance unit, current unit not changed\n");
    #endif
  }
}
//=============================================================================
int
tracker::get_angle_units(void)
{
  return angle_units_;
}
//=============================================================================
void
tracker::set_hemisphere(double x, double y, double z)
{
  if (get_logging())
    return;

  //Set hemisphere of operation with vector
  char buf[MAX_LENGTH];
  sprintf(buf, "H%d,%lf,%lf,%lf\r", STATION, x, y, z);
  port->send_comport(buf);
}
//=============================================================================
void
tracker::set_default_hemisphere(void)
{
  if (get_logging())
    return;

  //Set hemisphere of operation with vector
  set_hemisphere(DEF_HEM_X, DEF_HEM_Y, DEF_HEM_Z);
}
//=============================================================================
int
tracker::get_hemisphere(double* x, double* y, double* z)
{
  if (get_logging())
    return -1;

  //double tx, ty, tz;
  double values[3];
  //Read hemisphere of operation
  char buf[MAX_LENGTH];
  sprintf(buf, "H%d\r", STATION);
  port->send_comport(buf);
  port->recv_comport(buf, MAX_LENGTH);


  parse_stupid_string(buf, 3, values, 3);
//  if (sscanf(buf, "%*s %lf %lf %lf", &tx, &ty, &tz) != 3)
//    return -1;

  *x = values[0];
  *y = values[1];
  *z = values[2];
  return 1;
}
//=============================================================================
int
tracker::get_pose(double* x, double* y, double* z, double* yaw, double* pitch, double* roll)
{
  //Returns current pose
  char buf[MAX_LENGTH];
  char item[8];
  int i,j,k;
  double values[6];
  double dist_mult, angle_mult;

  //Get pose from comport
  port->send_comport("P");
  port->recv_comport(buf, MAX_LENGTH);


  //parse_stupid_string(buf, 6, values, 3);

  
  //Parse: Ignore first three chars, then read seven chars per value
  for (i=0;i<6;i++)
  {
    k=0;
    for (j=(i*7)+3;j<(i*7)+10;j++)
      item[k++]=buf[j];
    item[k] = '\0';

    if (sscanf(item, "%lf", &(values[i])) != 1)
    {
      #if TraceErrors
      printf("tracker: error parsing string\n");
      #endif
      return -1;
    }
  }  

  //Assign values to non-NULL parameters
  if (x!=NULL)
    *x = values[0]*DIST_MULT(dist_units_);
  if (y!=NULL)
    *y = values[1]*DIST_MULT(dist_units_);
  if (z!=NULL)
    *z = values[2]*DIST_MULT(dist_units_);
  if (yaw!=NULL)
    *yaw = (NORM_DEG(values[3] - by))*ANGLE_MULT(angle_units_);
  if (pitch!=NULL)
    *pitch = (NORM_DEG(values[4] - bp))*ANGLE_MULT(angle_units_);
  if (roll!=NULL)
    *roll = (NORM_DEG(values[5] - br))*ANGLE_MULT(angle_units_);

  return 1;
}

//=============================================================================

/*int
tracker::get_multi_pose(int num, double* x, double** y, double** z, double** yaw, double** pitch, double** roll)
{

  for(int i=0;i<num;i++){
    printf("q: %d\n",i);  
    get_pose(&x[i],y[i],z[i],yaw[i],pitch[i],roll[i]);
  }
  printf("q\n");

}*/

//=============================================================================
int
tracker::get_default_log_filename(char* filename, int max_length)
{
  if (get_logging())
    return -1;

  int length;
  char temp_filename[128];
  struct tm* date;
  time_t current_time;

  //Get time, use to make default log filename
  current_time = time(NULL);
  date = localtime(&current_time);
  sprintf(temp_filename, "%02d%02d%02d-%02d_%02d_%02d.log",
          date->tm_year-100, date->tm_mon+1, date->tm_mday, date->tm_hour,
          date->tm_min, date->tm_sec);

  length = strlen(temp_filename);

  if (length > max_length)
  {
    #if TraceErrors
    printf("FusionMonitor: get_default_log_file_name: default log file name too long\n");
    #endif
    return -1;
  }

  strcpy(filename, temp_filename);
  return 1;
}
//=============================================================================
int
tracker::get_default_xml_filename(char* filename, int max_length)
{
  if (get_logging())
    return -1;

  int length;
  char temp_filename[128];
  struct tm* date;
  time_t current_time;

  //Get time, use to make default log filename
  current_time = time(NULL);
  date = localtime(&current_time);
  sprintf(temp_filename, "%02d%02d%02d-%02d_%02d_%02d.xml",
          date->tm_year-100, date->tm_mon+1, date->tm_mday, date->tm_hour,
          date->tm_min, date->tm_sec);

  length = strlen(temp_filename);

  if (length > max_length)
  {
    #if TraceErrors
    printf("FusionMonitor: get_default_log_file_name: default log file name too long\n");
    #endif
    return -1;
  }

  strcpy(filename, temp_filename);
  return 1;
}
//=============================================================================
void
tracker::start_logging(char* filename)
{
  if (get_logging())
    return;

  char logfilename[128];
  FILE* file;
  timer t;
  unsigned long time;
  double x,y,z,ya,pi,ro;

  if (filename == NULL)
  if (get_default_log_filename(logfilename, 128) == -1)
  {
    #if TraceErrors
    printf("tracker: start_logging: could not get default log filename\n");
    #endif
    return;
  }
  
  
  child_pid_ = fork();
  if (child_pid_ == -1)
  {
    #if TraceErrors
    printf("tracker: start_logging: error creating child process\n");
    #endif
    return;
  }

  set_logging(1);

  if (child_pid_ == 0)
  {

    file = fopen(logfilename, "a+");
    if (file == NULL)
    {
      #if TraceErrors
      printf("tracker: start_logging: error opening file\n");
      #endif
    }  
      
    t.timer_reset();
    
    while(get_logging())
    {
      get_pose(&x,&y,&z,&ya,&pi,&ro);
      time = t.timer_read();
      fprintf(file, "%lu %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf\n", time, x, y, z, ya, pi, ro);
    }

    fclose(file);
    exit(0);
  }
}
//=============================================================================
void
tracker::start_logging_xml(char* filename)
{
  if (get_logging())
    return;

  char logfilename[128];
  FILE* file;
  timer t;
  unsigned long time;
  double x,y,z,ya,pi,ro;
  //double *x, *y, *z, *ya, *pi, *ro;
  //int num = 2;

  if (filename == NULL)
  if (get_default_xml_filename(logfilename, 128) == -1)
  {
    #if TraceErrors
    printf("tracker: start_logging: could not get default log filename\n");
    #endif
    return;
  }
  
  
  child_pid_ = fork();
  if (child_pid_ == -1)
  {
    #if TraceErrors
    printf("tracker: start_logging: error creating child process\n");
    #endif
    return;
  }

  set_logging(1);

  if (child_pid_ == 0)
  {

    file = fopen(logfilename, "a+");
    if (file == NULL)
    {
      #if TraceErrors
      printf("tracker: start_logging: error opening file\n");
      #endif
    }  
      
    t.timer_reset();
    
    fprintf(file,"<demo>\n");
    while(get_logging())
    {
      
      get_pose(&x,&y,&z,&ya,&pi,&ro);
      //get_multi_pose(num,&x,&y,&z,&ya,&pi,&ro);

      time = t.timer_read();
      fprintf(file,"<point>\n");
      fprintf(file,"<time>%.3lf</time>\n",((double)time) /1000 );
      fprintf(file,"<end-effector>");
      fprintf(file, "<position>%.2lf;%.2lf;%.2lf</position><YPR>%.2lf;%.2lf;%.2lf</YPR>", x, y, z, ya, pi, ro);
      //fprintf(file, "<position>%.2lf;%.2lf;%.2lf</position><YPR>%.2lf;%.2lf;%.2lf</YPR>", x, y, z, ya, pi, ro);
      printf("%lu %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf\n",time,  x, y, z, ya, pi, ro);
      /*for(int i=0;i<num;i++){
	printf("%lu %.2lf %.2lf %.2lf %.2lf %.2lf %.2lf\n",time,  x[i], y[i], z[i], ya[i], pi[i], ro[i]);
      }*/
      fprintf(file,"</end-effector>\n");
      fprintf(file,"</point>\n");
    }
    fprintf(file,"</demo>\n");
    fclose(file);
    exit(0);
  }
}
//=============================================================================
void
tracker::stop_logging(void)
{
  //Parent process gets this call
  if (!get_logging())
    return;
  set_logging(0);
  
  //Send interrupt to child
  kill(child_pid_, SIGALRM);
}
//=============================================================================
void
tracker::set_logging(int val)
{
  logging_ = val;
}
//=============================================================================
int
tracker::get_logging(void)
{
  return logging_;
}
//=============================================================================
void
catch_child(int sig_num)
{
  //Parent process gets this call
  int child_status;
  wait(&child_status);
}
//=============================================================================
void
catch_interrupt(int sig_num)
{
  //Child process gets this
  tracker::set_logging(0);
}
//=============================================================================
