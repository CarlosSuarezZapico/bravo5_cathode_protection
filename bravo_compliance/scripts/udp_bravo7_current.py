#!/usr/bin/env python3
from bplprotocol import BPLProtocol, PacketID
import time

import socket

if __name__ == '__main__':

    device_id = 0x06  # Joint B

    MANIPULATOR_IP_ADDRESS = '192.168.2.4'
    MANIPULATOR_PORT = 6789
    manipulator_address = (MANIPULATOR_IP_ADDRESS, MANIPULATOR_PORT)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    while(1):
        
        # Send to a position of 0.5 radians
        sock.sendto(BPLProtocol.encode_packet(device_id, PacketID.CURRENT, BPLProtocol.encode_floats([-750.0])), manipulator_address)

  
