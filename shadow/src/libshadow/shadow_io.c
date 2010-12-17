/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Alexander Skoglund, Karolinska Institute
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

#include "shadow_base.h"
#include "shadow_io.h"

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

void shadowDeviceSetParams(shadow_spcu_device_p dev)
{
  struct termios ctio;
 
  tcgetattr(dev->fd, &ctio); /* save current port settings */
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

  //  ctio.c_cflag = B19200; //cBaudrate(brate);

  // Input flags - Turn off input processing
  // convert break to null byte, no CR to NL translation,
  // no NL to CR translation, don't mark parity errors or breaks
  // no input parity check, don't strip high bit off,
  // no XON/XOFF software flow control  
  //ctio.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |		      
  //		      INLCR | PARMRK | INPCK | ISTRIP | IXON);
  ctio.c_iflag = 0;

  ctio.c_cflag |= CS8;      // 8 bit
  ctio.c_cflag |= CREAD;    // 

  ctio.c_cflag |= CLOCAL;   //
  ctio.c_cflag |= CRTSCTS;  //

  ctio.c_oflag = 0;
  ctio.c_lflag = 0;
  ctio.c_cc[VTIME] = 1; // Time out after 0.1 s
  ctio.c_cc[VMIN] = 0;  // blocking read until 0 chars received 
  

  tcflush(dev->fd, TCIFLUSH);
  tcsetattr(dev->fd, TCSANOW, &ctio);
  printf("Successfully executed shadowDeviceSetParams\n");
}

void shadowDeviceSetBaudrate(shadow_spcu_device_p dev, int brate)
{
  struct termios ctio;

  tcgetattr(dev->fd, &ctio); /* save current port settings */

  cfsetispeed(&ctio, (speed_t) cBaudrate(brate));
  cfsetospeed(&ctio, (speed_t) cBaudrate(brate));

  tcflush(dev->fd, TCIFLUSH);
  tcsetattr(dev->fd, TCSANOW, &ctio);
}

int shadowDeviceConnectPort(shadow_spcu_device_p dev)
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
  dev->fd  = open((dev->ttyport), (O_RDWR | O_NOCTTY) );
  if ( dev->fd < 0 )
    {
      printf("Error when open %s\n", dev->ttyport);
      return (-1);
    }
  else
   printf("Successfully opened %s\n", dev->ttyport); //return (-1)  
   shadowDeviceSetParams(dev);
   return (dev->fd);
}

void shadowDeviceClosePort(shadow_spcu_device_p dev)
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

int shadowSendCommandEchoAscii(int fd, unsigned char *cmd, int len)
{
  int written = 0;
  int nChars = len;
  int nRead;
  int i;
  unsigned char echo[1];

  // DEBUGGING
  //printf("In shadowSendCommand\n");
  //printf("send fd%d\n ", fd);
  //for(i=0;i< len;i++)
  //  printf("%d ",cmd[i]);
  //printf("\n ");
  //*******************

  
  /*i=0;
  while (nChars > 0 ) {
    written = write(fd, &(cmd[i]),len);
    nChars = nChars - written;
    if (written < len){
      i = i + written;
      usleep(1000);
    }
    }*/

  i=0;
  while (nChars > 0 ) {
    //written = write(fd, &(cmd[i]),len);
    // Write one char
    written = write(fd, &(cmd[i]),1);
    nChars = nChars - written;
    if (written < len){
      i = i + written;
      //usleep(1000);
      // Wait for the echo
      nRead = read(fd, &echo[0], 1);
      printf("echo %.2x\n ", echo[0]);
    }
  }
  // Wait for the LF + OK
  for(i=0;i<3;i++){
    nRead = read(fd, &echo[0], 1);
    printf("cs %.2x\n ", echo[0]);
  }
  return 1;
}

int shadowSendCommandEcho(int fd, unsigned char *cmd, int len)
{
  int written = 0;
  int nChars = len;
  int nRead;
  int i;
  unsigned char echo[1];

  // DEBUGGING
  //printf("In shadowSendCommand\n");
  //printf("send fd%d\n ", fd);
  //for(i=0;i< len;i++)
  //  printf("%d ",cmd[i]);
  //printf("\n ");
  //*******************

  
  /*i=0;
  while (nChars > 0 ) {
    written = write(fd, &(cmd[i]),len);
    nChars = nChars - written;
    if (written < len){
      i = i + written;
      usleep(1000);
    }
    }*/

  i=0;
  while (nChars > 0 ) {
    //written = write(fd, &(cmd[i]),len);
    // Write one char
    written = write(fd, &(cmd[i]),1);
    nChars = nChars - written;
    if (written < len){
      i = i + written;
      //usleep(1000);
      // Wait for the echo
      nRead = read(fd, &echo[0], 1);
      printf("echo %.2x\n ", echo[0]);
    }
  }
  // Wait for the checksum
  nRead = read(fd, &echo[0], 1);
  printf("cs %.2x\n ", echo[0]);
  return 1;
}

int shadowSendCommand(int fd, unsigned char *cmd, int len)
{
  int written = 0;
  int nChars = len;
  int nRead;
  int i;
  unsigned char echo[1];

  // DEBUGGING
  //printf("In shadowSendCommand\n");
  //printf("send fd%d\n ", fd);
  //for(i=0;i< len;i++)
  //  printf("%d ",cmd[i]);
  //printf("\n ");
  //*******************

  
  /*i=0;
  while (nChars > 0 ) {
    written = write(fd, &(cmd[i]),len);
    nChars = nChars - written;
    if (written < len){
      i = i + written;
      usleep(1000);
    }
    }*/

  i=0;
  while (nChars > 0 ) {
    //written = write(fd, &(cmd[i]),len);
    // Write one char
    written = write(fd, &(cmd[i]),1);
    nChars = nChars - written;
    if (written < len){
      i = i + written;
      usleep(1000);
    }
    /*else
    {
      // Wait for the echo
      nRead = read(fd, &echo[0], 1);
      printf("echo %x\n ", echo[0]);
      }*/
  }
  // Wait for the checksum
  //nRead = read(fd, &echo[0], 1);
  //printf("cs %x\n ", echo[0]);
  return 1;
}



int shadowGetAnswer(int fd, unsigned char* buf, int len, int cs_flag) //, unsigned char *cmd)
{
  int loop=0;
  int i, nread;
  int w = 1;
  int pos;
  //unsigned char lbuf[MAX_NUM_CHAR];
  unsigned char checksum=0;
  
  
  //printf("In shadowGetAnswer \n");
  nread=0;
  pos = 0;
  while (loop < MAX_NUM_LOOPS) {
    w = bytesWaiting(fd);
    //printf("len  %d\n ",len);
    if (len > 0) {
      nread = read(fd, &(buf[pos]), len);
      pos = pos + nread;
      // j = nread;
      //printf("nread  %d:  ",nread);      
      if( pos == len )  // Are we done reading ?
	{
	  if( cs_flag ){ 
	    for(i=1;i < len-1;i++){ // Compute checksum
	      //printf("%.2x ",buf[i]);
	      checksum = checksum + buf[i]; //    checksum = (checksum && 0x0f)+ inbuf[i];
	      //printf("\t ch %.4x\n ", checksum);
	    }
	    //checksum = checksum && 0xff;
	    //printf("\t ch %.2x\n ", checksum);
	    //printf("buf[%d] ch %x\n ",len-1, buf[len-1]);
	    if( buf[len-1] == checksum)
	      return len;
	    else
	      return -1;
	  }
	  return len;
	}
      else
	{
	  usleep(100);
	  loop++;
	}
      
    } 
    else {
      usleep(100);
      loop++;
    }
  }
  return 0;
}

