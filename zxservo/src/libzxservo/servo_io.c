/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Alexander Skoglund, Karolinska Institute
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Karolinska Institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/serial.h>

#include "servo_base.h"
#include "servo_io.h"

//#define IO_DEBUG


/*int cStopSize(int numbits)
{
  if (numbits == 2)
    return (CSTOPB);
  else
    return (0);
}

int cFlowControl(int flowcontrol)
{
  if (flowcontrol)
    return (CRTSCTS);  //    return (CNEW_RTSCTS);//return (CRTSCTS);  
  else
    return (CLOCAL);
}

int cParity(enum PARITY_TYPE par)
{
  if (par != N) {
    if (par == O)
      return (PARENB | PARODD);
    else
      return (PARENB);
  } else
    return (0);
}*/

int cBaudrate(int baudrate)
{
  switch (baudrate) {
  case 0:
    return (B0);
    break;
  case 300:
    return (B300);
    break;
  case 600:
    return (B600);
    break;
  case 1200:
    return (B1200);
    break;
  case 2400:
    return (B2400);
    break;
  case 4800:
    return (B4800);
    break;
  case 9600:
    return (B9600);
    break;
  case 19200:
    return (B19200);
    break;
  case 38400:
    return (B38400);
    break;
  case 57600:
    return (B57600);
    break;
  case 115200:
    return (B115200);
    break;
  case 500000:
    /* to use 500k you have to change the entry of B460800 in you kernel:
     /usr/src/linux/drivers/usb/serial/ftdi_sio.h:
     ftdi_8U232AM_48MHz_b460800 = 0x0006    */
    return (B460800);
    break;
  default:
    return (B9600);
    break;
  }
}


long bytesWaiting(int sd)
{

  long available = 0;
  if (ioctl(sd, FIONREAD, &available) == 0)
    return available;
  else
    return -1;
}

void servoDeviceSetParams(servo_device_p dev)
{
  struct termios options;

  // set everything to 0
  //bzero(&options, sizeof(options));

  // again set everything to 0
  tcgetattr(dev->fd, &options); /* Get the current port settings */
  
  bzero(&options, sizeof(options));
 
  options.c_cflag = B2400;

  cfsetispeed(&options, B2400);
  cfsetospeed(&options, B2400);
  
  /*
   * Enable the receiver and set local mode...
   */
  
  options.c_cflag |= (CLOCAL | CREAD | CS8);
  
 
  
  //options.c_cflag &= ~CSIZE; /* Mask the character size bits */
  //options.c_cflag |= CS8;    /* Select 8 data bits */
  
  //options.c_cflag &= ~PARENB;
  //options.c_cflag &= ~CSTOPB;
  //options.c_cflag &= ~CSIZE;
  //options.c_cflag |= CS8;

  //options.c_cflag &= ~CRTSCTS; //CNEW_RTSCTS; /* Disable hardware flow control */

  // RAW data 
  //options.c_lflag &= ~(ICANON | ECHO | ISIG);
  //options.c_lflag &= ~(ECHO | ECHOE);

  //options.c_iflag |= (INPCK | ISTRIP);
  options.c_iflag = IGNPAR;

  // No software flow control
  //options.c_iflag &= ~(IXON | IXOFF | IXANY);

  options.c_oflag = 0;
  //options.c_oflag &= ~OPOST;
  
  options.c_lflag = 0;
  options.c_cc[VTIME] = 1; // Time out after 0.1 s
  options.c_cc[VMIN] = 5;  // blocking read until 0 chars received 

  /*
   * Set the new options for the port...
   */
  
  tcflush(dev->fd, TCIFLUSH);
  tcsetattr(dev->fd, TCSAFLUSH, &options);
  tcsetattr(dev->fd, TCSANOW, &options);

  //struct termios ctio;
 
  //tcgetattr(dev->fd, &ctio); /* save current port settings */
  /*
  ctio.c_iflag = iSoftControl(dev->swf) | iParity(dev->parity);
  ctio.c_oflag = 0;
  ctio.c_cflag = CREAD | cFlowControl(dev->hwf || dev->swf)
                       | cParity(dev->parity)
                       | cDataSize(dev->databits)
                       | cStopSize(dev->stopbits);
  ctio.c_lflag = 0;
  ctio.c_cc[VTIME] = 0; // inter-character timer unused 
  ctio.c_cc[VMIN] = 0;  // blocking read until 0 chars received 

  cfsetispeed(&ctio, (speed_t) cBaudrate(dev->baud));
  cfsetospeed(&ctio, (speed_t) cBaudrate(dev->baud));
  */

  //ctio.c_cflag = cBaudrate(2400);

  // Input flags - Turn off input processing
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation, don't mark parity errors or breaks
  // no input parity check, don't strip high bit off,
  // no XON/XOFF software flow control  
  //ctio.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |		      
  //		      INLCR | PARMRK | INPCK | ISTRIP | IXON);
  //ctio.c_iflag = 0;

  //ctio.c_cflag |= CS8;      // 8 bit
  //ctio.c_cflag |= CREAD;    // 

  //ctio.c_cflag |= CLOCAL;   //
  //ctio.c_cflag |= CRTSCTS;  //

  //ctio.c_oflag = 0;
  //ctio.c_lflag = 0;
  //ctio.c_cc[VTIME] = 1; // Time out after 0.1 s
  //ctio.c_cc[VMIN] = 1;  // blocking read until 0 chars received 
  

  //tcflush(dev->fd, TCIFLUSH);
  //tcsetattr(dev->fd, TCSANOW, &ctio);


  printf("Successfully executed servoDeviceSetParams\n");
}

void servoDeviceSetBaudrate(servo_device_p dev, int brate)
{
  struct termios ctio;

  tcgetattr(dev->fd, &ctio); /* save current port settings */

  cfsetispeed(&ctio, (speed_t) cBaudrate(brate));
  cfsetospeed(&ctio, (speed_t) cBaudrate(brate));

  tcflush(dev->fd, TCIFLUSH);
  tcsetattr(dev->fd, TCSANOW, &ctio);
}

int servoDeviceConnectPort(servo_device_p dev)
{
  fprintf(stderr, "\nset device:\n");
  fprintf(stderr, "   port           = %s\n", dev->ttyport);
  fprintf(stderr, "   baud           = %d\n", dev->baud);
  /*  fprintf(stderr, "   params         = %d%s%d\n", dev->databits,
                                          dev->parity == N ? "N": dev->parity == O ? "O" : "E",
                                          dev->stopbits);
					  fprintf(stderr, "   hardwareflow   = %d\n", dev->hwf);*/
  //if ((dev->fd = open((dev->ttyport), (O_RDWR | O_NOCTTY), 0)) < 0)
  //  return (-1);
  dev->fd  = open((dev->ttyport), (O_RDWR | O_NOCTTY ) );
  if ( dev->fd < 0 )
    {
      printf("Error when open %s\n", dev->ttyport);
      return (-1);
    }
  else
   printf("Successfully opened %s\n", dev->ttyport); //return (-1)  
   servoDeviceSetParams(dev);
   return (dev->fd);
}

void servoDeviceClosePort(servo_device_p dev)
{

  /*if (!is_comport_open()){
    //printf("ComPort: Warning: Cannot close port, it is not open\n");    
    return -1;
    }*/


  //Purge buffers
  if (tcflush(dev->fd, TCIOFLUSH) == -1)
  {
    printf("ComPort: Warning: Cannot purge port buffers\n");
  }

  //Restore old COM settings
  /*  if (tcsetattr(port->handle,TCSANOW,&port->old_settings) == -1)
  {
    #if TraceErrors
    printf("ComPort: Warning: Cannot set port settings\n");
    #endif
    }*/

  close(dev->fd);
  dev->fd = -1;
}

int servoSendReceiveCommand(int fd, unsigned char *cmd, int len, unsigned char *echo, int ans)
{
  int written = 0;
  int nChars = len;
  int nRead;
  int i,j;
  unsigned char lecho[20];
  int loop=0;
  int w = 1;
  int pos;
  //int ans = 3;
  int done = 0;
  int ok;

  for(i=0;i< 20;i++)
    echo[i] =0;

  
  // DEBUGGING
  printf("\nIn servoSendCommandEchoAscii\n");
  //printf("send fd%d\n ", fd);
  for(i=0;i< len;i++)
    printf("%.2x ",cmd[i]);
  printf("\n ");
  //*******************

  
  i=0;
  while (nChars > 0 ) {
    written = write(fd, &(cmd[i]),len);
    nChars = nChars - written;
    if (written < len){
      i = i + written;
      usleep(100);
    }
  }

  usleep(1500);

  nRead=0;
  pos = 0;
  while (done == 0 ) {
    w = bytesWaiting(fd);
    if (w > 0) {
      printf("w:  %d:  ",w);      
      nRead = read(fd, &(lecho[pos]), len);
      pos = pos + nRead;
      printf("nRead: %d pos: %d\n",nRead,pos);  
    }
    if( pos == (len+ans) )  // Are we done reading ?
      done = 1;
    usleep(100);
    loop++;
    //printf("l %d ",loop);
    if( loop > MAX_NUM_LOOPS )
      break;
  }
  ok = 1;
  if( done == 1)
    {
      printf("Done ");
      for(i=0;i<len;i++){ 
	if( lecho[i] != cmd[i])
	  ok= 0;
      }
      if( ok == 1)
	{
	  j = 0;
	  for( ;i<len+ans;i++){
	    echo[i-len]=lecho[i];
	    printf("%.2x",echo[i-len]);
	  }
	  printf("\n");
	}
      else
	{
	  return 0;
	}
      
    }


  for(i=len;i<len+ans;i++){  
    printf("%.2x ", echo[i]);
  }
  printf("\n");
 
  return 1;
}



