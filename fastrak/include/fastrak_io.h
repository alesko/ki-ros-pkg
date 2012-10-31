//=============================================================================
//fastrak_io.h
//=============================================================================
#ifndef fastrak_io_h_DEFINED
#define fastrak_io_h_DEFINED
//=============================================================================


//=============================================================================
//Defines
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

#if ((WINDOWS && LINUX) || (!WINDOWS && !LINUX))
#error "ERROR: define either windows or linux"
#endif

#define TraceErrors 1

#define FIRST_PORT 1
#define LAST_PORT  4
#define IN_BUF_SIZE   255
#define OUT_BUF_SIZE  255

#define PARITY_NONE 0
#define PARITY_ODD  1
#define PARITY_EVEN 2
#define PARITY_DEFAULT 0

#define DATA_BITS_5 5
#define DATA_BITS_6 6
#define DATA_BITS_7 7
#define DATA_BITS_8 8
#define DATA_BITS_DEFAULT 8

#define STOP_BITS_1 1
#define STOP_BITS_2 2
#define STOP_BITS_DEFAULT 1

#define FLOW_CTRL_NONE 0
#define FLOW_CTRL_HARDWARE 1
#define FLOW_CTRL_XONXOFF  2
#define FLOW_CTRL_DEFAULT 0

#if LINUX
#define DEFAULT_BAUD B9600
#define MAX_BAUD B115200
#define BAUD_RATE(x)\
 (((x)==0)       ? B0      :\
  ((x)==50)      ? B50     :\
  ((x)==75)      ? B75     :\
  ((x)==110)     ? B110    :\
  ((x)==134)     ? B134    :\
  ((x)==150)     ? B150    :\
  ((x)==200)     ? B200    :\
  ((x)==300)     ? B300    :\
  ((x)==600)     ? B600    :\
  ((x)==1200)    ? B1200   :\
  ((x)==1800)    ? B1800   :\
  ((x)==2400)    ? B2400   :\
  ((x)==4800)    ? B4800   :\
  ((x)==9600)    ? B9600   :\
  ((x)==19200)   ? B19200  :\
  ((x)==38400)   ? B38400  :\
  ((x)==57600)   ? B57600  :\
  ((x)==115200)  ? B115200 :\
  ((x)==230400)  ? B230400 :\
  ((x)==460800)  ? B460800 :\
  ((x)==500000)  ? B500000 :\
  ((x)==576000)  ? B576000 :\
  ((x)==921600)  ? B921600 :\
  ((x)==1000000) ? B1000000:\
  ((x)==1152000) ? B1152000:\
  ((x)==1500000) ? B1500000:\
  ((x)==2000000) ? B2000000:\
  ((x)==2500000) ? B2500000:\
  ((x)==3000000) ? B3000000:\
  ((x)==3500000) ? B3500000:\
  ((x)==4000000) ? B4000000:DEFAULT_BAUD)
#else
#define DEFAULT_BAUD CBR_9600
#define BAUD_RATE(x)\
 (((x)==110)     ? CBR_110   :\
  ((x)==300)     ? CBR_300   :\
  ((x)==1200)    ? CBR_1200  :\
  ((x)==2400)    ? CBR_2400  :\
  ((x)==4800)    ? CBR_4800  :\
  ((x)==9600)    ? CBR_9600  :\
  ((x)==19200)   ? CBR_19200 :\
  ((x)==38400)   ? CBR_38400 :\
  ((x)==56000)   ? CBR_56000 :\
  ((x)==57600)   ? CBR_57600 :\
  ((x)==115200)  ? CBR_115200:\
  ((x)==128000)  ? CBR_128000:\
  ((x)==256000)  ? CBR_256000:DEFAULT_BAUD)
#endif

//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <ctype.h>
#include <string.h>

#if LINUX
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#else
#include <windows.h>
#endif
//=============================================================================


//=============================================================================
//Structs
//=============================================================================
typedef struct{
  int num;
  char name[16];
  unsigned long baud;
  #if LINUX
  int             handle;
  struct termios  settings;
  struct termios  old_settings;
  #else
  HANDLE          handle;
  DCB             settings;
  DCB             old_settings;
  COMMTIMEOUTS    timeouts;
  COMMTIMEOUTS    old_timeouts;
  DWORD           event_mask;
  DWORD           old_event_mask;
  #endif
} ComPort;
//=============================================================================


//=============================================================================
//Class
//=============================================================================
class comport
{
  public:
  
  comport();
  ~comport();

  int open_comport(int num, unsigned long baud, int stopbits, int databits);//int parity, int flowctrl)
  int close_comport(void);
  int send_comport(char* data);
  int recv_comport(char* data, int max_length);
  int is_comport_open(void);
  int peek_comport(void);
  
  private:
  ComPort* port;
  
};
//=============================================================================


//=============================================================================
#endif //ifndef fastrak_io_DEFINED
//=============================================================================
