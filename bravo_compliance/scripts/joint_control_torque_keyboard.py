
"""bplprotocol/examples/requesting_joints_positions_udp.py
Example to request the Joint positions from the arm.
"""

from bplprotocol import BPLProtocol, PacketID, PacketReader
import socket
import keyboard
import time

# Configuration
device_id = 0x02  # Joint B
MANIPULATOR_IP_ADDRESS = '192.168.1.51'
MANIPULATOR_PORT = 6789
manipulator_address = (MANIPULATOR_IP_ADDRESS, MANIPULATOR_PORT)

# UDP socket setup
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Torque control variables
TORQUE_MAGNITUDE = 1.5  # Adjust as needed
current_torque = 0.0

def send_torque_command(value):
    packet = BPLProtocol.encode_packet(device_id, 0x0B, BPLProtocol.encode_floats([value]))
    sock.sendto(packet, manipulator_address)
    print(f"Sent torque: {value:.2f}")

print("Controls:\n  W - Positive Torque\n  S - Negative Torque\n  SPACE - Zero Torque\n  Q - Quit")

try:
    while True:
        new_torque = current_torque

        if keyboard.is_pressed('w'):
            new_torque = TORQUE_MAGNITUDE

        elif keyboard.is_pressed('s'):
            new_torque = -TORQUE_MAGNITUDE

        elif keyboard.is_pressed('space'):
            new_torque = 0.0

        elif keyboard.is_pressed('q'):
            print("Exiting...")
            send_torque_command(0.0)
            break

        # Send only if torque changed
        if new_torque != current_torque:
            current_torque = new_torque
            send_torque_command(current_torque)

        time.sleep(0.05)

except KeyboardInterrupt:
    send_torque_command(0.0)
    print("\nProgram interrupted. Torque reset to 0.")
