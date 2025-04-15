#include <stdio.h>
#include <string>
#include "dx2lib.h"

int main(int argc, char* argv[])
{
  if(argc < 2){
    printf("usage:%s com_port\n", argv[0]);
    exit(1);
  }
  TDeviceID dev;
  TErrorCode err;
  char *COMPort = argv[1];
  int Baudrate = 1000000;

  if ((dev = DX2_OpenPort(COMPort, Baudrate)))
  {
    printf("Open success\n");
    for (int i = 0; i <= 252; ++i)
    {
      if (DX2_Ping(dev, i, &err))
      {
        printf("\rFound     ID=%3d [$%04X]\n", i, err);
      }
      else
      {
        printf("\rNot found ID=%3d [$%04X]\r", i, err);
      }
    }

    DX2_ClosePort(dev);
  }
  else
  {
    printf("Failed to open %s\n", COMPort);
  }

  printf("\nFin\n");
}
