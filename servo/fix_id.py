import scservo_sdk as scs

PORT = "/dev/ttyACM0"
BAUDRATE = 1000000
FROM_ID = 6
TO_ID = 1

port_handler = scs.PortHandler(PORT)
packet_handler = scs.PacketHandler(1)

port_handler.openPort()
port_handler.setBaudRate(BAUDRATE)

# Confirm the motor is there
model, result, error = packet_handler.ping(port_handler, FROM_ID)
if result != scs.COMM_SUCCESS:
    print(f"Motor ID {FROM_ID} not found. Make sure only the shoulder_pan is connected.")
    port_handler.closePort()
    exit(1)

print(f"Found motor at ID {FROM_ID} (model={model}). Changing to ID {TO_ID}...")

# Disable torque first (required to write to EEPROM)
packet_handler.write1ByteTxRx(port_handler, FROM_ID, 40, 0)  # Torque_Enable = 0
packet_handler.write1ByteTxRx(port_handler, FROM_ID, 55, 0)  # Lock = 0

# Write new ID
result, error = packet_handler.write1ByteTxRx(port_handler, FROM_ID, 5, TO_ID)
if result == scs.COMM_SUCCESS:
    print(f"Success! Motor ID changed from {FROM_ID} to {TO_ID}.")
    print("Now reconnect the gripper cable and run scan_servos.py to verify.")
else:
    print(f"Failed to change ID. result={result}, error={error}")

port_handler.closePort()
