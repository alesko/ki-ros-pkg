#include "fastrak_io.h"

using namespace std;

int main(int argc, char **argv)
{

  char buf[128];
  int i;

  comport port;

  //std::string fastrak_dev = argv[1];
  int fastrak_dev = 0;
  int fastrak_baudrate = 115200;

  if (argc > 3 | argc < 2 )
  {
    printf("Usage: comport test DEVICE BAUD_RATE\n");
    return 1;
  }
  if (argc == 3)
  {
    fastrak_dev = atoi(argv[1]);
    fastrak_baudrate = atoi(argv[2]);
  }
  if (argc == 2)
  {
    fastrak_dev = atoi(argv[1]);
  }


  if (port.open_comport(fastrak_dev, 115200, 1, 8) == -1)
  {
    printf("couldn't open comport\n");
    return -1;
  }

  for (i=0;i<100;i++)
  {
    if (port.send_comport("P\n") < 1)
      break;
      
    if (port.recv_comport(buf, 128) < 1)
      break;
      
    printf("received: <%s>", buf);
  }
  
  return 0;
}
