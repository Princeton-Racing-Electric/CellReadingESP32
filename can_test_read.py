import can

bus = can.interface.Bus(
    interface='slcan', channel='COM4', bitrate=500000, timeout=5)


if (bus):
    print("CAN bus active")


try:
    for msg in bus:
        print(msg)

except (KeyboardInterrupt):
    print("Shutting down bus")
    bus.shutdown()
