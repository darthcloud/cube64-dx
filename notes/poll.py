#!/usr/bin/env python3
#
# Script for polling N64/GC SI bus devices
#
# This script uses the serial bridge and pool in loops
# for the buttons status.
#
# It currently supports N64 controllers, N64 mouses & GameCube controllers.
#
# --Jacques Gagnon <darthcloud@gmail.com>
#

from bus import Bus
from collections import namedtuple, OrderedDict
import struct, time, os, sys

bmap = namedtuple('status', 'name mask color')

identity_req = namedtuple('identity_req', 'cmd')
status_req = namedtuple('status_req', 'cmd')
read_req = namedtuple('read_req', 'cmd address')
write_req = namedtuple('write_req', 'cmd address data')
dol_status_req = namedtuple('dol_status_req', 'cmd unknown rumble')
dol_wb_assoc_req = namedtuple('dol_wb_assoc_req', 'cmd id')

identity_resp = namedtuple('identity_resp', 'id info')
status_resp = namedtuple('status_resp', 'buttons x_axis y_axis')
dol_status_resp = namedtuple('dol_status_resp', 'buttons x_axis y_axis cx_axis cy_axis l_trigger r_trigger')

RED='\x1b[1;91m'
GREEN='\x1b[1;92m'
YELLOW='\x1b[1;93m'
BLUE='\x1b[1;94m'
MAGENTA='\x1b[1;95m'
CYAN='\x1b[1;96m'
LGRAY='\x1b[1;37m'
DGRAY='\x1b[1;90m'
END='\x1b[0m'

IDENTIFY = 0x00
STATUS = 0x01
READ = 0x02
WRITE = 0x03
DOL_STATUS = 0x40
WB_INIT = 0x4E

MOUSE = 0x02
NUS = 0x05
DOL = 0x09
WB_DOWN = 0xA8
WB_AUTH = 0xE9
WB_ASSOC = 0xEB

EMPTY = 0x00
OCCUPY = 0x01
EMPTIED = 0x02
INSERT = 0x03

BUTTON = {NUS:OrderedDict(
             [('buttons',[bmap('A',0x8000,BLUE),
                          bmap('B',0x4000,GREEN),
                          bmap('Z',0x2000,LGRAY),
                          bmap('St',0x1000,RED),
                          bmap('Up',0x0800,LGRAY),
                          bmap('Dn',0x0400,LGRAY),
                          bmap('Lt',0x0200,LGRAY),
                          bmap('Rt',0x0100,LGRAY),
                          bmap('L',0x0020,LGRAY),
                          bmap('R',0x0010,LGRAY),
                          bmap('CUp',0x0008,YELLOW),
                          bmap('CDn',0x0004,YELLOW),
                          bmap('CLt',0x0002,YELLOW),
                          bmap('CRt',0x0001,YELLOW)]),
              ('x_axis', [bmap('X',0xFF,LGRAY)]),
              ('y_axis', [bmap('Y',0xFF,LGRAY)])]),

          DOL:OrderedDict(
             [('buttons',[bmap('St',0x1000,LGRAY),
                          bmap('Y',0x0800,LGRAY),
                          bmap('X',0x0400,LGRAY),
                          bmap('B',0x0200,RED),
                          bmap('A',0x0100,CYAN),
                          bmap('L',0x0040,LGRAY),
                          bmap('R',0x0020,LGRAY),
                          bmap('Z',0x0010,MAGENTA),
                          bmap('Up',0x0008,LGRAY),
                          bmap('Dn',0x0004,LGRAY),
                          bmap('Rt',0x0002,LGRAY),
                          bmap('Lt',0x0001,LGRAY)]),
              ('x_axis', [bmap('X',0xFF,LGRAY)]),
              ('y_axis', [bmap('Y',0xFF,LGRAY)]),
              ('cx_axis', [bmap('CX',0xFF,YELLOW)]),
              ('cy_axis', [bmap('CY',0xFF,YELLOW)]),
              ('l_trigger', [bmap('AL',0xFF,LGRAY)]),
              ('r_trigger', [bmap('AR',0xFF,LGRAY)])])}

class Bus(Bus):
    def identify(self):
        reply = self.bridge.write(bytes([IDENTIFY]), 3)[1]

        if reply[0] == MOUSE:
          return {'system':NUS, 'type':'mouse'}
        elif reply[0] == NUS:
          if reply[2] == EMPTY:
            return {'system':NUS, 'type':'controller', 'slot':'Empty  '}
          elif reply[2] == OCCUPY:
            return {'system':NUS, 'type':'controller', 'slot':'Occupy '}
          elif reply[2] == EMPTIED:
            return {'system':NUS, 'type':'controller', 'slot':'Emptied'}
          elif reply[2] == INSERT:
            return {'system':NUS, 'type':'controller', 'slot':'Insert '}
          else:
            print("Unknown N64 controller slot state: {}".format(reply))
            sys.exit()
        elif reply[0] == DOL:
          return {'system':DOL, 'type':'controller'}
        elif reply[0] == WB_DOWN:
          return {'system':WB_DOWN, 'type':'wavebird'}
        elif reply[0] == WB_AUTH:
          return {'system':WB_AUTH, 'type':'wavebird', 'id':reply[-2:]}
        elif reply[0] == WB_ASSOC:
          return {'system':DOL, 'type':'wavebird'}
        else:
          print("Unknown device identity: {}".format(reply))
          sys.exit()

    def status(self, system):
        if system == NUS:
          reply = self.bridge.write(bytes([STATUS]), 4)[1]
          return status_resp._make(struct.unpack('>H2b', reply))
        elif system == DOL:
          reply = self.bridge.write(struct.pack(">BH", DOL_STATUS, 0x0300), 8)[1]
          return dol_status_resp._make(struct.unpack('>H6B', reply))
        else:
          print("Unknown system ID: {}".format(system))
          sys.exit()

    def wavebird_init(self, id):
        return self.bridge.write(struct.pack(">BBB", WB_INIT, (id[0] | 0x20) & 0x10, id[1]), 3)[1]

def poll():
    os.system('setterm -cursor off')

    interface = Bus()
    device = interface.identify()

    time.sleep(0.02)

    while device['system'] == WB_DOWN:
      device = interface.identify()
      time.sleep(1)

    if device['system'] == WB_AUTH:
      interface.wavebird_init(device['id'])

    try:
      while 1:
        device = interface.identify()
        time.sleep(0.02)
        status = interface.status(device['system'])
        for field, values in BUTTON[device['system']].items():
          for value in values:
            if value.mask != 0xFF:
              print("{}{}{} ".format(value.color if getattr(status, field) & value.mask else DGRAY, value.name, END), end='')
            else:
              print("{}{}:{:+03X}{} ".format(value.color, value.name, getattr(status, field), END), end='')
        if 'slot' in device:
          print("slot:{}".format(device['slot']), end='')
        print("\r", end='')
        time.sleep(0.02)
    except KeyboardInterrupt:
      pass

    os.system('setterm -cursor on')
    print("")

if __name__ == "__main__":
    poll()

### The End ###
