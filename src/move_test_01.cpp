#include <stdio.h>
#include <string>
#include <unistd.h>
#include "dx2lib.h"

int main(int argc, char *argv[])
{
  char port[] = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AI0282XE-if00-port0";
  TDeviceID dev = DX2_OpenPort(port, 1000000);
  uint8_t ids[] = {1, 2};
  const int num = 2;
  double time = 1;
  double margin = 0.01;
  for (int i = 0; i < num; ++i)
  {
    PDXL_ModelInfo p = DXL_GetModelInfo(dev, ids[i]);
    if (p->modelno != 0)
    {
      printf("[%3d] %s ($%04X) %d\n", ids[i], p->name, p->modelno, p->devtype);
    }
  }
  DXL_SetDriveModesEquival(dev, ids, num, 0x04);
  DXL_SetOperatingModesEquival(dev, ids, num, 3);
  DXL_SetTorqueEnablesEquival(dev, ids, num, true);
  {
    double angles[] = {90, -90};
    DXL_SetGoalAnglesAndTime2(dev, ids, angles, 2, time);
    usleep((time + margin) * 1000 * 1000);
  }
  {
    double angles[] = {-90, 90};
    DXL_SetGoalAnglesAndTime2(dev, ids, angles, 2, time);
    usleep((time + margin) * 1000 * 1000);
  }
  {
    double angles[] = {0, 0};
    DXL_SetGoalAnglesAndTime2(dev, ids, angles, 2, time);
    usleep((time + margin) * 1000 * 1000);
  }
  DXL_SetTorqueEnablesEquival(dev, ids, num, false);
  DX2_ClosePort(dev);
  printf("\nFin\n");
}
