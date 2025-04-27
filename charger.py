#!/usr/bin/env python3
import can
import struct
import time

bus = can.interface.Bus(interface='slcan',
                        channel='COM6',
                        bitrate=500000,
                        timeout=0.2)           # non-blocking read

print("Bus:", bus.channel_info)

CTRL_ID = 0x2FF
CHARGER_ENABLE             = 0x01
CHARGER_POWER_REFERENCE    = 0x03E8   # 1000  → 1000/10 = 100 %
CHARGER_MAXDCVOLTLIMIT     = 0x047C   # 1148 → 114.8 V ceiling = 4.1 * 28
CHARGER_MAXDCCURRLIMIT     = 0x01F4   #  500  →   50.0 A ceiling

ctrl_frame = can.Message(
    arbitration_id=CTRL_ID,
    data=[
        CHARGER_ENABLE,
        CHARGER_POWER_REFERENCE        & 0xFF,  # LSB
        CHARGER_POWER_REFERENCE   >> 8 & 0xFF,  # MSB
        CHARGER_MAXDCVOLTLIMIT         & 0xFF,
        CHARGER_MAXDCVOLTLIMIT    >> 8 & 0xFF,
        CHARGER_MAXDCCURRLIMIT         & 0xFF,
        CHARGER_MAXDCCURRLIMIT    >> 8 & 0xFF
    ],
    is_extended_id=False
)

print("Sending:", ctrl_frame)

STATUS1_ID = 0x305          # charger → host
SEND_PERIOD = 1.0           # seconds between command refreshes
next_send  = time.time()

try:
    while True:
        now = time.time()

        # refresh command once per second (many chargers time-out)
        if now >= next_send:
            bus.send(ctrl_frame)
            next_send = now + SEND_PERIOD

        # read any incoming frame (non-blocking)
        msg = bus.recv()
        if msg is None:
            continue

        # decode only Status-1 frames (0x305)
        # might want to parse error which is 307 to send a message with power reference of 0, voltage of 0 and current of 0?
        if msg.arbitration_id == STATUS1_ID and len(msg.data) >= 7:
            d = msg.data

            # bytes: 5 = LSB, 6 = MSB
            raw_voltage = (d[6] << 8) | d[5]      # big-endian 16-bit
            voltage_v   = raw_voltage / 10.0

            raw_current = (d[4] << 8) | d[3]
            current_a   = raw_current / 10.0

            status_code = d[0]   # 1 = IDLE, 2 = CHARGE, 3/4 errors

            print(f"[{msg.timestamp:.3f}] "
                  f"Status={status_code}  "
                  f"Vdc={voltage_v:6.1f} V  "
                  f"Idc={current_a:5.1f} A")

except KeyboardInterrupt:
    print("\nShutting down bus…")
finally:
    bus.shutdown()
