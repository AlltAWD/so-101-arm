import scservo_sdk as scs

PORT = "/dev/ttyACM1"
BAUDRATE = 1000000

port_handler = scs.PortHandler(PORT)
packet_handler = scs.PacketHandler(1)

port_handler.openPort()
port_handler.setBaudRate(BAUDRATE)

print(f"Scanning for Feetech servos on {PORT} at {BAUDRATE} baud...")
print("ID  | Status")
print("----|-------")

found = []
for id_ in range(1, 20):
    model, result, error = packet_handler.ping(port_handler, id_)
    if result == scs.COMM_SUCCESS:
        found.append(id_)
        print(f" {id_:2d} | FOUND (model={model})")

port_handler.closePort()
print()
if found:
    print(f"Found {len(found)} servo(s) at IDs: {found}")
    if len(found) < 6:
        print("WARNING: Expected 6 servos — some may have duplicate IDs or lost connection.")
    elif found != list(range(1, 7)):
        print("WARNING: IDs are not the expected 1-6.")
    else:
        print("All 6 servos found with correct IDs 1-6.")
else:
    print("No servos found. Check connection and power.")
