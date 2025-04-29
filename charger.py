#!/usr/bin/env python3
import can
import struct
import time

bus = can.interface.Bus(interface='slcan',
                        channel='COM4',
                        bitrate=500000,
                        timeout=0.2)  # non-blocking read

print("Bus:", bus.channel_info)

CTRL_ID = 0x2FF
STATUS1_ID = 0x305
STATUS2_ID = 0x306
ERRORS_ID  = 0x307

CHARGER_ENABLE             = 0x01
CHARGER_POWER_REFERENCE    = 0x03E8   # 1000 → 100.0%
CHARGER_MAXDCVOLTLIMIT     = 0x047C   # 114.8 V
CHARGER_MAXDCCURRLIMIT     = 0x00E6   # 23.0 A

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

SEND_PERIOD = 0.5  # seconds between command refreshes
next_send = time.time()

try:
    while True:
        now = time.time()

        # refresh control frame once per second
        if now >= next_send:
            bus.send(ctrl_frame)
            next_send = now + SEND_PERIOD

        # non-blocking read
        msg = bus.recv()
        if msg is None:
            continue

        d = msg.data

        if msg.arbitration_id == STATUS1_ID and len(d) >= 7:
            status_code = d[0]
            raw_current = (d[4] << 8) | d[3]
            current_a = raw_current / 10.0

            raw_voltage = (d[6] << 8) | d[5]
            voltage_v = raw_voltage / 10.0

            print(f"[{msg.timestamp:.3f}] Status1 "
                  f"Status={status_code}  "
                  f"Vdc={voltage_v:6.1f} V  "
                  f"Idc={current_a:5.1f} A")

        elif msg.arbitration_id == STATUS2_ID and len(d) >= 7:
            primary_temp = struct.unpack('b', d[0:1])[0]
            secondary_temp = struct.unpack('b', d[1:2])[0]

            mains_voltage = (d[3] << 8) | d[2]
            mains_voltage_v = mains_voltage  # already in V

            max_power = (d[5] << 8) | d[4]
            available_power = d[6] / 2.0  # %

            print(f"[{msg.timestamp:.3f}] Status2 "
                  f"PrimaryTemp={primary_temp}°C  "
                  f"SecondaryTemp={secondary_temp}°C  "
                  f"MainsVoltage={mains_voltage_v}V  "
                  f"MaxPower={max_power}W  "
                  f"AvailablePower={available_power:.1f}%")

        elif msg.arbitration_id == ERRORS_ID and len(d) >= 3:
            # Parse error bits
            error_flags = []

            error_map = {
                0: "DCOVS (DC Overvoltage Shutdown)",
                1: "SCICOMMFAIL (SCI Comms Failure)",
                2: "HIGHMAINS (High Mains Shutdown)",
                3: "LOWMAINS (Low Mains Shutdown)",
                4: "HIGHTEMP (High Temperature Shutdown)",
                5: "LOWTEMP (Low Temperature Shutdown)",
                6: "CURRLIM (Current Limit Derating)",
                8: "MODFAIL (Transformer Failure)",
                10: "DCUVS (DC Undervoltage Shutdown)",
                14: "CNTCOMMFAIL (Communication Timeout)"
            }

            byte0 = d[0]
            byte1 = d[1]
            byte2 = d[2]

            for bit, description in error_map.items():
                byte_index = bit // 8
                bit_index  = bit % 8
                if byte_index == 0 and (byte0 & (1 << bit_index)):
                    error_flags.append(description)
                elif byte_index == 1 and (byte1 & (1 << bit_index)):
                    error_flags.append(description)
                elif byte_index == 2 and (byte2 & (1 << bit_index)):
                    error_flags.append(description)

            if error_flags:
                print(f"[{msg.timestamp:.3f}] Errors: {', '.join(error_flags)}")
            else:
                print(f"[{msg.timestamp:.3f}] Errors: (no errors)")

except KeyboardInterrupt:
    print("\nShutting down bus…")
finally:
    bus.shutdown()
