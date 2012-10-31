//=============================================================================
//timer.hpp
//=============================================================================
#ifndef _timer_h_DEFINED
#define _timer_h_DEFINED
//=============================================================================


//=============================================================================
//OS
//=============================================================================
#if ((defined __linux__) || (defined __CYGWIN__))
#ifndef LINUX
#define LINUX 1
#endif
#ifndef WINDOWS
#define WINDOWS 0
#endif
#else
#ifndef LINUX
#define LINUX 0
#endif
#ifndef WINDOWS
#define WINDOWS 1
#endif
#endif
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include <sys/time.h>
#include <time.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#ifndef LINUX
#include <winsock.h>
#include <windows.h>
#endif

#include <stdio.h>
#include <string.h>
//=============================================================================


//=============================================================================
//Function Prototypes
//=============================================================================
class timer
{
  public:
    timer();
    ~timer();

    unsigned long timer_read(void);
    void          timer_reset(void);
    void          timer_sleep(unsigned long ms);

  private:
    unsigned long start_time;
    struct timeval time;

    #ifndef LINUX
    DWORD dword;
    #endif

};
//=============================================================================


//=============================================================================
#endif //#ifndef _timer_h_DEFINED
//=============================================================================
