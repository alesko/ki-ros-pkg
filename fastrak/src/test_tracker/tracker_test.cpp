#include "fastrak_tracker.h"

int main(void)
{
  double x,y,z,pi,ya,ro;

  char ch;
  tracker t;

  /*
  printf("Setting defaults...\n");
  t.restore_system_defaults();
  printf("Done!\n\n\n");
  */

  /*
  printf("Storing settings to EEPROM...\n");
  t.store_current_to_eeprom();
  printf("Done!\n\n\n");
  */

  /*
  printf("Restarting tracker - this takes %.1f seconds\n", (float)RESTART_TIME/1000);
  t.restart_tracker();
  printf("Done!\n\n\n");
  */

  /*
  //Set and check hemisphere
  printf("Setting and checking hemisphere...\n");
  t.set_hemisphere(0,0,1.0); //This is the default anyway
  if (t.get_hemisphere(&x,&y,&z) == -1)
    printf("error1\n");
  else
    printf("Hemisphere: (%.2lf,%.2lf,%.2lf)\n", x, y, z);
  printf("Done!\n\n\n");
  */

  /*
  //Set and check units
  printf("Setting and checking units...\n");
  t.set_dist_units(DIST_UNIT_MM);
  t.set_angle_units(ANGLE_UNIT_DEG);
  printf("Using %s for XYZ and %s for angles\n", DIST_UNIT_NAME(t.get_dist_units()), ANGLE_UNIT_NAME(t.get_angle_units()));
  */

  //Interactively set up coordinate system
  /*printf("Interactively set up origin? (y/n - then enter)\n");
  ch = getchar();
  if (ch == 'y')
  {
    getchar();
    t.setup_origin(INTERACTIVE);
  }
  else
  getchar();


  printf("Interactively set up boresight? (y/n - then enter)\n");
  ch = getchar();
  if (ch == 'y')
  {
    getchar();
    t.setup_boresight(INTERACTIVE);
  }
  else
  getchar();*/


  /*
  printf("Non-interactively setting up origins...\n");
  //Non-interactively set up coordinate system
  //This could be used if you want to use your own interface to the calibration (e.g. via a GUI)
  t.setup_origin(STEP_0); //Indicate start - place probe at proposed origin
  printf("Place probe at desired xyz origin, then hit enter...\n");
  getchar();
  t.setup_origin(STEP_1); //Indicate this is origin - now place probe on X axis
  printf("Place probe 20-40cm along the positive X axis, then hit enter...\n");
  getchar();
  t.setup_origin(STEP_2); //Indicate this was X - now place probe on Y axis
  printf("Place probe 20-40cm along the positive Y axis, then hit enter...\n");
  getchar();
  t.setup_origin(STEP_3); //Indicate this was Y - done!
  printf("Done!\n");

  t.setup_boresight(STEP_0); //Indicate start - place probe with desired orientation
  printf("Place probe at desired origin orientation, then hit enter...");
  getchar();
  t.setup_boresight(STEP_1); //Could also just use t.set_boresight_to_current();
  printf("Done!\n");
  */

  /*
  //Check boresight
  printf("Checking boresight...\n");
  if (t.get_boresight(&ya,&pi,&ro) == -1)
    printf("error3\n");
  else
    printf("Boresight: (%.2lf,%.2lf,%.2lf)\n", ya, pi, ro);
  */

  /*
  printf("Reading position...  Hit ctrl-c to quit.\n");
  while(1)
  {
    if (t.get_pose(&x,&y,&z,&ya,&pi,&ro) == -1)
      printf("error4\n");
    else
      printf("(x,y,z,yaw,pitch,roll): (%5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf, %5.2lf)\n", x, y, z, ya, pi, ro);
  }
  */

  printf("Hit enter to start logging...\n");
  getchar();
  printf("Starting to log to default filename, hit enter to stop...\n");
  t.start_logging_xml(NULL);
  timer ti;
  getchar();

  t.stop_logging();
  printf("Done\n");
  return 0;
}
