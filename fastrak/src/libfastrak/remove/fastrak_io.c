/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
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
 *   * Neither the name of the Robert Bosch nor the names of its
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
#include "fastrak_io.h"

//#define IO_DEBUG

int iParity(enum PARITY_TYPE par)
{
  if (par == N)
    return (IGNPAR);
  else
    return (INPCK);
}

int iSoftControl(int flowcontrol)
{
  if (flowcontrol)
    return (IXON);
  else
    return (IXOFF);
}

int cDataSize(int numbits)
{
  switch (numbits) {
  case 5:
    return (CS5);
    break;
  case 6:
    return (CS6);
    break;
  case 7:
    return (CS7);
    break;
  case 8:
    return (CS8);
    break;
  default:
    return (CS8);
    break;
  }
}

int cStopSize(int numbits)
{
  if (numbits == 2)
    return (CSTOPB);
  else
    return (0);
}

int cFlowControl(int flowcontrol)
{
  if (flowcontrol)
    return (CRTSCTS);
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
}

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


void fastrakDeviceSetParams(polhemus_faskrak_device_p dev)
{
  struct termios ctio;

  tcgetattr(dev->fd, &ctio); /* save current port settings */

  ctio.c_iflag = iSoftControl(dev->swf) | iParity(dev->parity);
  ctio.c_oflag = 0;
  ctio.c_cflag = CREAD | cFlowControl(dev->hwf || dev->swf)
                       | cParity(dev->parity)
                       | cDataSize(dev->databits)
                       | cStopSize(dev->stopbits);
  ctio.c_lflag = 0;
  ctio.c_cc[VTIME] = 0; /* inter-character timer unused */
  ctio.c_cc[VMIN] = 0;  /* blocking read until 0 chars received */

  cfsetispeed(&ctio, (speed_t) cBaudrate(dev->baud));
  cfsetospeed(&ctio, (speed_t) cBaudrate(dev->baud));

  tcflush(dev->fd, TCIFLUSH);
  tcsetattr(dev->fd, TCSANOW, &ctio);
}

void fastrakDeviceSetBaudrate(polhemus_fastrak_device_p dev, int brate)
{
  struct termios ctio;

  tcgetattr(dev->fd, &ctio); /* save current port settings */

  cfsetispeed(&ctio, (speed_t) cBaudrate(brate));
  cfsetospeed(&ctio, (speed_t) cBaudrate(brate));

  tcflush(dev->fd, TCIFLUSH);
  tcsetattr(dev->fd, TCSANOW, &ctio);
}

int fastrakDeviceConnectPort(polhemus_fastrak_device_p dev)
{
  fprintf(stderr, "\nset device:\n");
  fprintf(stderr, "   port   = %s\n", dev->ttyport);
  fprintf(stderr, "   baud   = %d\n", dev->baud);
  fprintf(stderr, "   params = %d%s%d\n", dev->databits,
                                          dev->parity == N ? "N": dev->parity == O ? "O" : "E",
                                          dev->stopbits);
  if ((dev->fd = open((dev->ttyport), (O_RDWR | O_NOCTTY), 0)) < 0)
    return (-1);
  //amtecDeviceSetParams(dev);
  fastrakDeviceSetParams(dev);
  return (dev->fd);
}
