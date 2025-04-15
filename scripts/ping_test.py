#!/usr/bin/python3
# -*- coding: utf-8 -*-


from dx2lib import *


def main():
    COMPort = '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_AI0282XE-if00-port0'
    Baudrate = 1000000

    dev = DX2_OpenPort(COMPort, Baudrate)
    if dev == None:
        print('Failed to open', COMPort)
        return
    err = c_uint16(0)

    DX2_SetTimeOutOffset(dev, 20)
    # ping
    print('>>DX2_Ping')
    for id in range(253):
        r = DX2_Ping(dev, id, err)
        print('\r ping = %d  err = %d     ' % (id, err.value), end='')
        if r:
            print('\n find', id)
            print(DXL_GetModelInfo(dev, id).contents.name.decode())
    print()
    DX2_ClosePort(dev)


if __name__ == '__main__':
    main()
