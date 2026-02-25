
from bplprotocol import BPLProtocol, PacketID
import socket
import keyboard
import time

# Configuration
device_id = 0x02  # Joint B
MANIPULATOR_IP_ADDRESS = '192.168.2.4'
MANIPULATOR_PORT = 6789
manipulator_address = (MANIPULATOR_IP_ADDRESS, MANIPULATOR_PORT)

# UDP socket setup
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Velocity control variables
velocity = 0.0
VELOCITY_STEP = 0.05
MAX_VELOCITY = 1.0
MIN_VELOCITY = -1.0

def send_velocity_command(value):
    packet = BPLProtocol.encode_packet(device_id, PacketID.VELOCITY, BPLProtocol.encode_floats([value]))
    sock.sendto(packet, manipulator_address)
    print(f"Sent velocity: {value:.2f}")

print("Controls:\n  W - Increase Velocity\n  S - Decrease Velocity\n  SPACE - Stop\n  Q - Quit")

try:
    while True:
        if keyboard.is_pressed('w'):
            velocity = min(MAX_VELOCITY, velocity + VELOCITY_STEP)
            send_velocity_command(velocity)
            time.sleep(0.1)

        elif keyboard.is_pressed('s'):
            velocity = max(MIN_VELOCITY, velocity - VELOCITY_STEP)
            send_velocity_command(velocity)
            time.sleep(0.1)

        elif keyboard.is_pressed('space'):
            velocity = 0.0
            send_velocity_command(velocity)
            time.sleep(0.1)

        elif keyboard.is_pressed('q'):
            print("Exiting...")
            send_velocity_command(0.0)
            break

except KeyboardInterrupt:
    send_velocity_command(0.0)
    print("\nProgram interrupted. Velocity reset to 0.")

