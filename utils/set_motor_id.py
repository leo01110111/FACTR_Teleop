#!/usr/bin/env python3
"""Fully configure the single Dynamixel motor on the bus for FACTR.

Usage: set_motor_id.py <new_id> [port]    (port defaults to /dev/ttyUSB0)
Does, for the ONE motor currently connected:
  1. set ID -> <new_id>
  2. set Baud Rate -> 4 Mbps
  3. set Return Delay Time -> 0
  4. verify it responds at 4 Mbps

Safety: refuses to write unless EXACTLY ONE motor responds cleanly
(connect one motor at a time, per the FACTR/ROBOTIS procedure).
"""
import sys
from dynamixel_sdk import PortHandler, PacketHandler

DEV = sys.argv[2] if len(sys.argv) > 2 else "/dev/ttyUSB0"
ADDR_TORQUE_ENABLE = 64
ADDR_ID = 7
ADDR_BAUD = 8
ADDR_RDT = 9
BAUD_VAL_4M = 6   # XC330: 0=9600,1=57600,2=115200,3=1M,4=2M,5=3M,6=4M,7=4.5M
SCAN_BAUDS = [57600, 9600, 115200, 1000000, 2000000, 3000000, 4000000]

new_id = int(sys.argv[1])
ph = PortHandler(DEV)
pk = PacketHandler(2.0)

clean = []
garbled = 0
for b in SCAN_BAUDS:
    if not ph.openPort():
        print("ERROR: cannot open", DEV); sys.exit(2)
    ph.setBaudRate(b)
    for sid in range(0, 21):
        model, res, err = pk.ping(ph, sid)
        if res == 0:
            clean.append((b, sid, model))
        elif res == -3002:
            garbled += 1
    ph.closePort()

if garbled and not clean:
    print(f"ERROR: bus garbled ({garbled} corrupt replies), no clean motor.")
    print("       Likely >1 motor (ID collision) or a wiring fault. Connect exactly ONE.")
    sys.exit(1)
if len(clean) == 0:
    print("ERROR: no motor detected. Connect exactly one motor and check power.")
    sys.exit(1)
if len({c[1] for c in clean}) > 1:
    print("ERROR: multiple motors detected:", clean, "- connect exactly ONE.")
    sys.exit(1)

baud, cur_id, model = clean[0]
print(f"Found 1 motor: current ID={cur_id}, baud={baud}, model={model}")

ph.openPort(); ph.setBaudRate(baud)
pk.write1ByteTxRx(ph, cur_id, ADDR_TORQUE_ENABLE, 0)  # EEPROM writes need torque off
r_id, _ = pk.write1ByteTxRx(ph, cur_id, ADDR_ID, new_id)
print("  set ID ->", new_id, ":", pk.getTxRxResult(r_id))
r_rdt, _ = pk.write1ByteTxRx(ph, new_id, ADDR_RDT, 0)
print("  set Return Delay Time -> 0 :", pk.getTxRxResult(r_rdt))
r_baud, _ = pk.write1ByteTxRx(ph, new_id, ADDR_BAUD, BAUD_VAL_4M)
print("  set Baud -> 4 Mbps :", pk.getTxRxResult(r_baud))
ph.closePort()

ok = 0
for _ in range(5):
    ph.openPort(); ph.setBaudRate(4000000)
    m, res, err = pk.ping(ph, new_id)
    ph.closePort()
    if res == 0:
        ok += 1
if ok:
    print(f"SUCCESS: ID {new_id} @ 4 Mbps, RDT 0  ({ok}/5 clean pings, model {m})")
else:
    print("WARN: writes sent but could not verify at 4 Mbps")
    sys.exit(1)
