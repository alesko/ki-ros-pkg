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
#include "amtec_io.h"

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


unsigned int CRC16(unsigned int crc, unsigned char data)
{
  const unsigned int tbl[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040};

    return ((crc & 0xFF00) >> 8) ^ tbl[(crc & 0x00FF) ^ (data & 0x00FF)];
}



void amtecDeviceSetParams(amtec_powercube_device_p dev)
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
  ctio.c_cc[VMIN] = 0; /* blocking read until 0 chars received */

  cfsetispeed(&ctio, (speed_t) cBaudrate(dev->baud));
  cfsetospeed(&ctio, (speed_t) cBaudrate(dev->baud));

  tcflush(dev->fd, TCIFLUSH);
  tcsetattr(dev->fd, TCSANOW, &ctio);
}

void amtecDeviceSetBaudrate(amtec_powercube_device_p dev, int brate)
{
  struct termios ctio;

  tcgetattr(dev->fd, &ctio); /* save current port settings */

  cfsetispeed(&ctio, (speed_t) cBaudrate(brate));
  cfsetospeed(&ctio, (speed_t) cBaudrate(brate));

  tcflush(dev->fd, TCIFLUSH);
  tcsetattr(dev->fd, TCSANOW, &ctio);
}

int amtecDeviceConnectPort(amtec_powercube_device_p dev)
{
  fprintf(stderr, "\nset device:\n");
  fprintf(stderr, "   port   = %s\n", dev->ttyport);
  fprintf(stderr, "   baud   = %d\n", dev->baud);
  fprintf(stderr, "   params = %d%s%d\n", dev->databits,
                                          dev->parity == N ? "N": dev->parity == O ? "O" : "E",
                                          dev->stopbits);
  if ((dev->fd = open((dev->ttyport), (O_RDWR | O_NOCTTY), 0)) < 0)
    return (-1);
  amtecDeviceSetParams(dev);
  return (dev->fd);
}

int waitForETX(int fd, unsigned char *buf, int *len)
{
  static int pos, loop, val;
#ifdef IO_DEBUG
  int i;
#endif
  pos = 0;
  loop = 0;
  while (loop < MAX_NUM_LOOPS) {
    val = bytesWaiting(fd);
    if (val > 0) {
      if(pos + val >= MAX_ACMD_SIZE) return (0);
      read(fd, &(buf[pos]), val);
#ifdef IO_DEBUG
      for(i=0;i<val;i++)
      fprintf(stderr, "[0x%s%x]", buf[pos+i]<16?"0":"", buf[pos+i]);
#endif
      if (buf[pos + val - 1] == B_ETX) {
        *len = pos + val - 1;
#ifdef IO_DEBUG
        fprintf(stderr, "\n");
#endif
        return (1);
      }
      pos += val;
    } else {
      usleep(1000);
      loop++;
    }
  }
#ifdef IO_DEBUG
  fprintf(stderr, "\n");
#endif
  return (0);
}

int waitForAnswer(int fd, unsigned char *buf, int *len)
{
  int loop = 0;
  *len = 0;
  //#ifdef IO_DEBUG
  printf("<?--- "); //fprintf(stderr, "<--- ");
  //#endif
  while (loop < MAX_NUM_LOOPS) {
    if (bytesWaiting(fd)) {
      read(fd, &(buf[0]), 1);
      //#ifdef IO_DEBUG
      printf("(0x%s%x)", buf[0]<16?"0":"", buf[0]); //fprintf(stderr, "(0x%s%x)", buf[0]<16?"0":"", buf[0]);
      //#endif
      //if (buf[0] == B_STX) {
      //if (buf[0] == S_TO_M ) {
      //  return (waitForETX(fd, buf, len));
      //}
    } else {
      usleep(1000);
      loop++;
    }
  }
  //#ifdef IO_DEBUG
  printf("\n"); //fprintf(stderr, "\n");
  //#endif
  return (0);
}

int schunkWaitForAnswer(int fd, unsigned char *cmd)
{
  static int val;
  int loop = 0;
  int rlen = 0;
  int i;
  unsigned char *inbuf;
  unsigned int crc = 0;
  unsigned char crc_hi, crc_low;


  //#ifdef IO_DEBUG
  //printf("<--- "); //fprintf(stderr, "<--- ");
  //#endif
  printf("In schunkWaitForAnswer \n");
  while (loop < MAX_NUM_LOOPS) {
    if (bytesWaiting(fd)) {
      //read(fd, &(inbuf[0]), 1);
      read(fd, &(cmd[0]), 1);
      if (cmd[0] == RESP_FROM_MOD ) {
	while (loop < MAX_NUM_LOOPS) {
	  if (bytesWaiting(fd)>1) {
	    read(fd, &(cmd[1]), 2); // ID + Len
	    rlen = cmd[2];
	    val = bytesWaiting(fd);
	    while( val < rlen+2 ){       // + CRC
	      val = bytesWaiting(fd); 
	    }
	    read(fd, &(cmd[3]), val);
	    for( i = 0; i < (rlen+2+3); i++)
	      printf("%2X:",cmd[i]);
	    printf("\n");  
	    // CRC16 check
	    for( i = 0; i < (rlen+4); i++){ // S2M + ID + D-Len
	      crc = CRC16(crc, cmd[i]);
	      if( i == rlen+2) crc_hi = crc;
	      if( i == rlen+3) crc_low = crc ;
	    }
	    //printf("crc (comp): %X:%X\n",crc_hi,crc_low);
	    //printf("crc (retr) :%X:%X\n",cmd[len+3],cmd[len+4]);
	    if( (crc_hi == cmd[rlen+3])&(crc_low == cmd[rlen+4]))
	      return 1;
	    else
	      return 0;
	  } else {
	    usleep(1000);
	    loop++;
	  }
	}
      }
    } else {
      usleep(1000);
      loop++;
    }
  }
  //#ifdef IO_DEBUG
  printf("\n"); //fprintf(stderr, "\n");
  //#endif
  return (0);
}

int writeData(int fd, unsigned char *buf, int nChars)
{
  int written = 0;
  while (nChars > 0) {
    written = write(fd, buf, nChars);
    if (written < 0) {
      return 0;
    } else {
      nChars -= written;
      buf += written;
    }
    usleep(1000);
  }
  return 1;
}


int amtecSchunkSendCommand(amtec_powercube_device_p dev, int id, unsigned char *cmd, int len)
{
  static unsigned char rcmd[MAX_ACMD_SIZE];
  unsigned int crc = 0;
  int i, ctr;

  // Group/ID
  rcmd[0] = 0x05;
  rcmd[1] = id;

  // Len
  rcmd[2] = len;

  // Cmd + data
  for (i = 0; i < len; i++) {
    rcmd[i+3] = cmd[i];  
  }
  // CRC
  for( i = 0; i < (len+3); i++){
    crc = CRC16(crc, rcmd[i]);
  }
  //printf("%2X:",crc);
  rcmd[i] =  crc;         // Low byte
  rcmd[i+1] = crc  >> 8;  // High byte

  ctr = len+5; // MSG ID D-Len +len + CRC

  // Debugging
  /*printf("In amtecSchunkSendCommand \n");
  for( i = 0; i < (len+5); i++)
    printf("%2X:",rcmd[i]);
  printf("\n");  
  printf("Len(ctr)%d:\n",ctr);*/

  if (writeData(dev->fd, rcmd, ctr)) {
    return (1);
  } else {
    return (0);
  }


}

int amtecSchunkGetAnswer(amtec_powercube_device_p dev, int id, unsigned char *cmd, unsigned char *buf)
{
  //#ifdef IO_DEBUG
  int i;
  int len;
  //endif
  
  if (schunkWaitForAnswer(dev->fd, buf)) {
    //ifdef IO_DEBUG
    //printf("<=== "); //fprintf(stderr, "<=== ");
    //for(i=0;i<len;i++)
    //  printf("[0x%s%x]", cmd[i]<16?"0":"", cmd[i]); //fprintf(stderr, "[0x%s%x]", cmd[i]<16?"0":"", cmd[i]);
    //printf("\n"); //fprintf(stderr, "\n");
    //endif
    //convertBuffer(cmd, len);
#ifdef IO_DEBUG
    printf("<=p= "); // printf(stderr,"<=p= ");
    for(i=0;i<len;i++)
      printf("[0x%s%x]", cmd[i]<16?"0":"", cmd[i]); //fprintf(stderr, "[0x%s%x]", cmd[i]<16?"0":"", cmd[i]);
    printf("\n"); //fprintf(stderr, "\n");
#endif
    //    printf("buf[3] = [0x%x], cmd[0] = [0x%x]", buf[3], cmd[0]);
    //printf("buf[1] = %d, id = %d", buf[1], id);
    if( (buf[3] == cmd[0]) & (buf[1] == id) )      
      return (1);
  } else {
    return (0);
  }
}

int amtecSendCommand(amtec_powercube_device_p dev, int id, unsigned char *cmd, int len)
{
  static int i, ctr, add;
  static unsigned char rcmd[MAX_ACMD_SIZE];
  static unsigned char bcc;
  static unsigned char umnr;
  static unsigned char lmnr;

#ifdef IO_DEBUG
  fprintf(stderr, "\n---> ");
  for(i=0;i<len;i++) {
    fprintf(stderr, "(0x%s%x)", cmd[i]<16?"0":"", cmd[i]);
  }
  fprintf(stderr, "\n");
#endif

  add = 0;
  lmnr = id & 7;
  lmnr = lmnr << 5;
  umnr = id >> 3;
  umnr = umnr | 4;
  for (i = 0; i < len; i++) {
    if ((cmd[i] == 0x02) || (cmd[i] == 0x03) || (cmd[i] == 0x10)) {
      add++;
    }
  }
  lmnr = lmnr + len;
  rcmd[0] = B_STX;
  rcmd[1] = umnr;
  rcmd[2] = lmnr;
  ctr = 3;
  for (i = 0; i < len; i++) {
    switch (cmd[i]) {
    case 0x02:
      rcmd[ctr] = 0x10;
      rcmd[++ctr] = 0x82;
      break;
    case 0x03:
      rcmd[ctr] = 0x10;
      rcmd[++ctr] = 0x83;
      break;
    case 0x10:
      rcmd[ctr] = 0x10;
      rcmd[++ctr] = 0x90;
      break;
    default:
      rcmd[ctr] = cmd[i];
    }
    ctr++;
  }
  bcc = id;
  for (i = 0; i < len; i++) {
    bcc += cmd[i];
  }
  bcc = bcc + (bcc >> 8);
  switch (bcc) {
  case 0x02:
    rcmd[ctr++] = 0x10;
    rcmd[ctr++] = 0x82;
    break;
  case 0x03:
    rcmd[ctr++] = 0x10;
    rcmd[ctr++] = 0x83;
    break;
  case 0x10:
    rcmd[ctr++] = 0x10;
    rcmd[ctr++] = 0x90;
    break;
  default:
    rcmd[ctr++] = bcc;
  }
  rcmd[ctr++] = B_ETX;

#ifdef IO_DEBUG
  fprintf(stderr, "-*-> ");
  for(i=0;i<ctr;i++) {
    fprintf(stderr, "(0x%s%x)", rcmd[i]<16?"0":"", rcmd[i]);
  }
  fprintf(stderr, "\n");
#endif

  if (writeData(dev->fd, rcmd, ctr)) {
    return (1);
  } else {
    return (0);
  }
}

void convertBuffer(unsigned char *cmd, int *len)
{
  int i, j;
  for (i = 0; i < *len; i++) {
    if (cmd[i] == B_DLE) {
      switch (cmd[i + 1]) {
      case 0x82:
        cmd[i] = 0x02;
        for (j = i + 2; j < *len; j++)
          cmd[j - 1] = cmd[j];
        (*len)--;
        break;
      case 0x83:
        cmd[i] = 0x03;
        for (j = i + 2; j < *len; j++)
          cmd[j - 1] = cmd[j];
        (*len)--;
        break;
      case 0x90:
        cmd[i] = 0x10;
        for (j = i + 2; j < *len; j++)
          cmd[j - 1] = cmd[j];
        (*len)--;
        break;
      }
    }
  }
}

int amtecGetAnswer(amtec_powercube_device_p dev, unsigned char *cmd, int *len)
{
#ifdef IO_DEBUG
  int i;
#endif
  if (waitForAnswer(dev->fd, cmd, len)) {
#ifdef IO_DEBUG
    fprintf(stderr, "<=== ");
    for(i=0;i<*len;i++)
      fprintf(stderr, "[0x%s%x]", cmd[i]<16?"0":"", cmd[i]);
    fprintf(stderr, "\n");
#endif
    convertBuffer(cmd, len);
#ifdef IO_DEBUG
    fprintf(stderr, "<=p= ");
    for(i=0;i<*len;i++)
      fprintf(stderr, "[0x%s%x]", cmd[i]<16?"0":"", cmd[i]);
    fprintf(stderr, "\n");
#endif
    return (1);
  } else {
    return (0);
  }
}
