//=============================================================================
//timer.cpp
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include "fastrak_timer.h"
//=============================================================================


//=============================================================================
//Functions
//=============================================================================
timer::timer(void)
{
  timer_reset();
}
//=============================================================================
timer::~timer(void)
{
}
//=============================================================================
void
timer::timer_reset(void)
{
  #ifdef LINUX
  gettimeofday(&time,NULL);
  #else
  dword = timeGetTime();
  time.tv_sec = dword/1000;
  time.tv_usec = dword%1000;
  #endif

  start_time = (unsigned long)((double)(time.tv_sec)*1000.0 + (double)(time.tv_usec)/1000.0);

}
//=============================================================================
unsigned long
timer::timer_read(void)
{
  #ifdef LINUX
  gettimeofday(&time,NULL);
  #else
  dword = timeGetTime();
  time.tv_sec = dword/1000;
  time.tv_usec = dword%1000;
  #endif

  //Return time - start_time
  return (unsigned long)( (double)(time.tv_sec)*1000.0 +
                          (double)(time.tv_usec)/1000.0 -
                          start_time);
}
//=============================================================================
void
timer::timer_sleep(unsigned long ms)
{
  #ifndef LINUX
  Sleep(ms);
  #else
  unsigned long start, elapsed;

  //Get time of day, initialise start_time (in s)
  start = timer_read();

  do
  {
    elapsed = timer_read();
  }
  while (elapsed < ms);
  #endif
}
//=============================================================================


