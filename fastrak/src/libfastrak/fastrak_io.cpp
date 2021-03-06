//=============================================================================
//fastrak_io.cpp
//=============================================================================


//=============================================================================
//Includes
//=============================================================================
#include "fastrak_io.h"
//=============================================================================


//=============================================================================
//Functions
//=============================================================================
comport::comport(void)
{
  port = (ComPort*)malloc(sizeof(ComPort));

  if (port == NULL)
  {
    #if TraceErrors
    printf("ComPort: Error: Could not allocate memory for port\n");
    #endif
    return;
  }

  #if LINUX
  port->handle = -1;
  #else
  port->handle = INVALID_HANDLE_VALUE;
  #endif
}
//=============================================================================
comport::~comport(void)
{
  free(port);
}
//=============================================================================
int
comport::open_comport(int num, unsigned long baud, int stopbits, int databits)//int parity, int flowctrl)
{
  int errors = 0;

  /*if (num < FIRST_PORT || num > LAST_PORT)
  {
    #if TraceErrors
    printf("ComPort: Error: Choose a port between %d and %d\n", FIRST_PORT, LAST_PORT);
    #endif
    return -1;
  }*/

  port->num = num;
  port->baud = baud;


  //---------------------------
  #if LINUX
  //---------------------------

  //Set port name
  //sprintf(port->name,"%s%d", "/dev/ttyS", num-1);
  sprintf(port->name,"%s%d", "/dev/ttyUSB",num);

  //Open port
  port->handle = open(port->name, O_RDWR | O_NOCTTY);

  //Check port
  if (port->handle < 0)
  {
    #if TraceErrors
    printf("ComPort: Error: Cannot open port\n");
    #endif
    return -1;
  }

  //Save old COM status
  if ((tcgetattr(port->handle, &port->old_settings)) == -1)
  {
    errors++;
    #if TraceErrors
    printf("ComPort: Warning: Cannot read port settings\n");
    #endif
  }

  //Edit COM status
  bzero(&port->settings,sizeof(port->settings));
  port->settings.c_iflag = IGNPAR;
  //port->settings.c_iflag |= IGNBRK|~BRKINT|IGNPAR|~PARMRK|~INPCK|
  //                          ~ISTRIP|~INLCR|~IGNCR|~ICRNL|~IUCLC|~IXON|
  //                          IXANY|~IXOFF|~IMAXBEL;
  port->settings.c_oflag = 0;
  //port->settings.c_oflag |= ~OPOST;
  port->settings.c_cflag = BAUD_RATE(baud)|CREAD|CLOCAL;

  switch(databits)
  {
    case DATA_BITS_5:
      port->settings.c_cflag |= CS5;
      break;
    case DATA_BITS_6:
      port->settings.c_cflag |= CS6;
      break;
    case DATA_BITS_7:
      port->settings.c_cflag |= CS7;
      break;
    case DATA_BITS_8:
      port->settings.c_cflag |= CS8;
      break;
    default:
      port->settings.c_cflag |= CS8;
      break;
  }

  switch(stopbits)
  {
    case STOP_BITS_1:
      break;
    case STOP_BITS_2:
      port->settings.c_cflag |= CSTOPB;
      break;
    default:
      break;
  }

  //port->settings.c_cflag |= BAUD_RATE|CS8|CSTOPB|CREAD|~PARENB|
  //                           ~PARODD|~HUPCL|CLOCAL|~LOBLK|~CRTSCTS;
  port->settings.c_lflag = 0;
  //port->settings.c_lflag |= ~ISIG|~ICANON|~XCASE|~ECHO|~IEXTEN;
  port->settings.c_cc[VTIME] = 1;
  port->settings.c_cc[VMIN] = IN_BUF_SIZE;

  //Purge buffers
  if (tcflush(port->handle, TCIOFLUSH) == -1)
  {
    errors++;
    #if TraceErrors
    printf("ComPort: Warning: Cannot purge port buffers\n");
    #endif
  }

  //Set new COM status
  if (tcsetattr(port->handle, TCSANOW, &port->settings) == -1)
  {
    errors++;
    #if TraceErrors
    printf("ComPort: Warning: Cannot set port status\n");
    #endif
  }

  //---------------------------
  #else //i.e. #if WINDOWS
  //---------------------------

  //Set port name
  sprintf(port->name,"%s%d", "COM", num);

  //Open port
  port->handle = CreateFile(port_name,GENERIC_READ|GENERIC_WRITE,0,NULL,OPEN_EXISTING,FILE_ATTRIBUTE_NORMAL,NULL);

  //Check port
  if (port->handle == INVALID_HANDLE_VALUE)
  {
    #if TraceErrors
    printf("ComPort: Error: Cannot open port\n");
    #endif
    return -1;
  }

  //Save old COM status
  if ((GetCommState(port->handle,&port->old_settings_dcb)) == 0)
  {
    errors++;
    #if TraceErrors
    printf("ComPort: Warning: Cannot read port settings\n");
    #endif
  }

  //Load old COM status
  if ((GetCommState(port->handle,&port->settings_dcb)) == 0)
  {
    errors++;
    #if TraceErrors
    printf("ComPort: Warning: Cannot set port settings\n");
    #endif
  }

  //Edit COM status
  port->settings_dcb.BaudRate = BAUD_RATE(baud);
  port->settings_dcb.fParity = FALSE;
  port->settings_dcb.fOutxCtsFlow = FALSE;
  port->settings_dcb.fOutxDsrFlow = FALSE;
  port->settings_dcb.fDtrControl = DTR_CONTROL_DISABLE;
  port->settings_dcb.fDsrSensitivity = FALSE;
  port->settings_dcb.fRtsControl = RTS_CONTROL_DISABLE;
  port->settings_dcb.Parity = NOPARITY;
  port->settings_dcb.EvtChar = '\r';

  switch(databits)
  {
    case DATA_BITS_5:
      port->settings_dcb.ByteSize = 5;
      break;
    case DATA_BITS_6:
      port->settings_dcb.ByteSize = 6;
      break;
    case DATA_BITS_7:
      port->settings_dcb.ByteSize = 7;
      break;
    case DATA_BITS_8:
      port->settings_dcb.ByteSize = 8;
      break;
    default:
      port->settings_dcb.ByteSize = 8;
      break;
  }

  switch(stopbits)
  {
    case STOP_BITS_1:
      port->settings_dcb.StopBits = ONESTOPBIT;
      break;
    case STOP_BITS_2:
      port->settings_dcb.StopBits = TWOSTOPBITS;
      break;
    default:
      port->settings_dcb.StopBits = ONESTOPBIT;
      break;
  }

  //Set new COM status
  if (SetCommState(port->handle,&port->settings_dcb) == 0)
  {
    errors++;
    #if TraceErrors
    printf("ComPort: Warning: Cannot set port settings\n");
    #endif
  }

  //Save old timeouts
  if (GetCommTimeouts(port->handle, &port->timeouts_old) == 0)
  {
    errors++;
    #if TraceErrors
    printf("ComPort: Warning: Cannot read port timeouts\n");
    #endif
  }

  //Load old timeouts
  if (SetCommTimeouts(port->handle, &port->timeouts) == 0)
  {
    errors++;
    #if TraceErrors
    printf("ComPort: Warning: Cannot set port timeouts\n");
    #endif
  }

  //Edit COM timeouts
  //port->timeouts.ReadIntervalTimeout = 10;
  port->timeouts.ReadIntervalTimeout = 50;
  //port->timeouts.ReadTotalTimeoutMultiplier = 1;
  //port->timeouts.ReadTotalTimeoutConstant = 10;
  //port->timeouts.WriteTotalTimeoutMultiplier = 0;
  //port->timeouts.WriteTotalTimeoutConstant = 0;

  //Set new COM timeouts
  if (SetCommTimeouts(port->handle, &port->timeouts) == 0)
  {
    errors++;
    #if TraceErrors
    printf("ComPort: Warning: Cannot set port timeouts\n");
    #endif
  }

  //Save old COM event mask
  if (GetCommMask(port->handle, &port->event_mask_old) == 0)
  {
    errors++;
    #if TraceErrors
    printf("ComPort: Warning: Cannot read port event mask\n");
    #endif
  }

  //Edit COM event mask
  port->event_mask = EV_RXFLAG;// | EV_RXCHAR;

  //Set event handler for COM port
  if (SetCommMask(port->handle, port->event_mask) == 0)
  {
    errors++;
    #if TraceErrors
    printf("ComPort: Warning: Cannot set port event mask\n");
    #endif
  }

  //Set COM port buffer sizes
  if (SetupComm(port->handle, (DWORD)(IN_BUF_SIZE), (DWORD)(OUT_BUF_SIZE)) == 0)
  {
    errors++;
    #if TraceErrors
    printf("ComPort: Warning: Cannot set port buffer sizes\n");
    #endif
  }

  //Purge input buffer
  if (PurgeComm(port->handle, PURGE_RXCLEAR) == 0)
  {
    errors++;
    #if TraceErrors
    printf("ComPort: Warning: Cannot purge port input buffer\n");
    #endif
  }

  //Purge output buffer
  if (PurgeComm(port->handle, PURGE_TXCLEAR) == 0)
  {
    errors++;
    #if TraceErrors
    printf("ComPort: Warning: Cannot purge port output buffer\n");
    #endif
  }
  //---------------------------
  #endif
  //---------------------------

  //Check for errors
  if (errors == 0)
    return 0;
  else
  {
    #if TraceErrors
    printf ("ComPort: Warning: There were problems configuring the port\n");
    #endif
    return -1;
  }
}
//=============================================================================
int
comport::close_comport(void)
{

  if (!is_comport_open())
  {
    #if TraceErrors
    printf("ComPort: Warning: Cannot close port, it is not open\n");
    #endif
    return -1;
  }

  //---------------------------
  #if LINUX
  //---------------------------

  //Purge buffers
  if (tcflush(port->handle, TCIOFLUSH) == -1)
  {
    #if TraceErrors
    printf("ComPort: Warning: Cannot purge port buffers\n");
    #endif
  }

  //Restore old COM settings
  if (tcsetattr(port->handle,TCSANOW,&port->old_settings) == -1)
  {
    #if TraceErrors
    printf("ComPort: Warning: Cannot set port settings\n");
    #endif
  }

  close(port->handle);
  port->handle = -1;

  //---------------------------
  #else
  //---------------------------

  //Purge input buffer
  if (PurgeComm(port->handle, PURGE_RXCLEAR) == 0)
  {
    #if TraceErrors
    printf("ComPort: Warning: Cannot purge port input buffer\n");
    #endif
  }

  //Purge output buffer
  if (PurgeComm(port->handle, PURGE_TXCLEAR) == 0)
  {
    #if TraceErrors
    printf("ComPort: Warning: Cannot purge port output buffer\n");
    #endif
  }

  //Restore old COM status
  if (SetCommState(port->handle,&port->old_settings) == 0)
  {
    #if TraceErrors
    printf("ComPort: Warning: Cannot set port settings\n");
    #endif
  }

  //Restore old COM timeouts
  if (SetCommTimeouts(port->handle, &port->old_timeouts) == 0)
  {
    #if TraceErrors
    printf("ComPort: Warning: Cannot set port timeouts\n");
    #endif
  }

  //Restore old event handler for COM port
  if (SetCommMask(port->handle, port->old_event_mask) == 0)
  {
    #if TraceErrors
    printf("ComPort: Warning: Cannot set port event mask\n");
    #endif
  }

  //Close port if it's open
  CloseHandle(port->handle);
  port->handle = INVALID_HANDLE_VALUE;

  //---------------------------
  #endif
  //---------------------------

  return 1;
}
//=============================================================================
int
comport::send_comport(char* data)
{
  #if WINDOWS
  DWORD wrote;
  #endif
  int length = strlen(data);

  //Discard empty messages
  if (length == 0)
    return 0;

  #if LINUX
  if (write(port->handle,data,length) < 0)
  #else
  if (WriteFile(port->handle, data, length, &wrote, 0) == 0)
  #endif
  {
    #if TraceErrors
    printf("ComPort: Error: Cannot write to port\n");
    #endif
    return 0;
  }

  #if LINUX
  return length;
  #else
  return wrote;
  #endif
}
//=============================================================================
int
comport::recv_comport(char* data, int max_length)
{
  char buf[IN_BUF_SIZE+1];
  int num, temp;


  num = 0;
  data[0] = '\0';

  /*
  //Return nothing if nothing there
  if (!peek_comport())
    return num;
  */

  do
  {

    #if LINUX
    temp = read(port->handle,buf,IN_BUF_SIZE);
    #else
    ReadFile(port->handle, buf, IN_BUF_SIZE, &temp, 0);
    #endif

    if (temp < 0)
    {
      #if TraceErrors
      printf("ComPort: Error: Cannot read from port\n");
      #endif
      return num;
    }

    num+= temp;

    if (num > max_length - 1)
    {
      #if TraceErrors
      printf("ComPort: Warning: Message truncated, receive buffer not long enough\n");
      #endif
      return num - temp;
    }

    strcat(data, buf);

  }while (temp >=IN_BUF_SIZE);

  data[num] = '\0';

  return num;
}
//=============================================================================
int
comport::is_comport_open(void)
{
  //---------------------------
  #if LINUX
  //---------------------------
  if (port->handle < 0)
    return 0;
  else
    return 1;
  //---------------------------
  #else
  //---------------------------
  if (port->handle == INVALID_HANDLE_VALUE)
    return 0;
  else
    return 1;
  //---------------------------
  #endif
  //---------------------------
}
//=============================================================================
int
comport::peek_comport(void)
{
  fd_set readfds;     //List of file descriptors for select to listen to
  struct timeval timeout;

  FD_ZERO(&readfds);  //Initialize readfds
  FD_SET(port->handle,&readfds);  //Set readfds to include comport
  timeout.tv_sec = 0;  //Set timeout to zero so select polls port
  timeout.tv_usec = 0;

  //Check if there is data on the port
  if (select(FD_SETSIZE,&readfds,(fd_set *)NULL,(fd_set *)NULL,&timeout) > 0)
    return 1;

  return 0;
}
//=============================================================================
