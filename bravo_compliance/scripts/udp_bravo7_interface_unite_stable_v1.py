#!/usr/bin/env python3
"""
    @author   Carlos Suarez Zapico carlossuarezzapico@gmail.com

    @internal
    Created  5-Oct-2021
    Modified for ROS2 16-Nov-2023
    Unite Project
    Description: UDP interface for bravo in python
"""

import rclpy
from rclpy.clock import Clock
from rclpy.node import Node

from bpl_msgs.msg import Packet, JointPosition, Generic, Packet, SingleFloat
from bplprotocol import PacketReader, BPLProtocol, PacketID

import socket
import threading
import copy
import struct
import crcmod

from geometry_msgs.msg import WrenchStamped, Twist
from sensor_msgs.msg import JointState

MANIPULATOR_IP_ADDRESS = '192.168.2.4'
MANIPULATOR_PORT = 6789

CRC8_FUNC = crcmod.mkCrcFun(0x14D, initCrc=0xFF, xorOut=0xFF) # what is this????
#FLOAT_PACKETS = Packets().get_float_packet_ids()


class udp_bravo7_bringup_unite_v6(Node):
    def __init__(self):
        super().__init__("udp_passthrough")

        self.device_ids = [1,2,3,4,5,6,7]
        self.device_ids_arm = [1,2,3,4,5,6]
        self.frequency_req = 300    # Frequency at which requests and motions commands are made. 
        self.time_delay_nano_req = (1/self.frequency_req)*10**9  # in nanoseconds

        self.frequency_recv = 200    # Frequency at which requests and motions commands are made. 
        self.time_delay_nano_recv = self.time_delay_nano_req;#(1/self.frequency_recv)*10**9  # in nanoseconds

        #ROS publishers
        self.other_pub               =   self.create_publisher(Generic,        'bravo/other_msg/recv',     10)
        self.wrench_pub              =   self.create_publisher(WrenchStamped,  '/bravo/fdb/Wrench',        10)
        self.robot_joint_state_pub   =   self.create_publisher(JointState,     '/bravo/fdb/joint_states',  10)    
        self.robot_joint_pos_pub     =   self.create_publisher(JointPosition,  '/bravo/fdb/JointPosition', 10)

        # Initialise ROS messages
        self.robot_joint_pos       =     JointPosition () 
        self.other_message         =     Generic()    
        self.wrench_msg            =     WrenchStamped() 
        self.arm_joint_state_msg   =     JointState()

        #ROS subscribers
        self.cmd_velocity_sub      =     self.create_subscription(SingleFloat, 'b7m_0' + '/cmd_velocity',   self.cmd_single_joint_velocity_callback,1)         
        self.cmd_LocalVelocity_sub =     self.create_subscription(Twist,         "/bravo/command/LocalVelocity", self.cmd_local_velocity_callback, 1) 
        self.cmd_jointPosition_sub =     self.create_subscription(JointPosition, "/bravo/command/JointPosition", self.cmd_joint_position_callback, 1)  
        
        self.robot_joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 7 = arm joints + gripper

        #JointStates msg
        self.arm_joint_state_msg.header.stamp = Clock().now().to_msg()
        self.arm_joint_state_msg.header.frame_id = 'base_link'
        self.arm_joint_state_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0];  # 6 joints
        self.arm_joint_state_msg.name = ['bravo_axis_g','bravo_axis_f','bravo_axis_e','bravo_axis_d', 'bravo_axis_c', 'bravo_axis_b']    
        self.arm_joint_dof = 6 # no gripper, moveit group
        self.q_joints_received = [False, False, False, False, False, False, False] # 7 = arm joints + gripper
        
        #Socket configuration
        self.ip_address = MANIPULATOR_IP_ADDRESS  # 708 controller . The ip to use TX2 is  192.168.2.3 but it is a slower controller
        self.port = MANIPULATOR_PORT
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(("", 0))
        #self.sock.settimeout(0) # what is this for???
        
        self.last_time_req = Clock().now().nanoseconds
        self.time_receiver = Clock().now().nanoseconds


        self.q_vel = [0, 0, 0, 0, 0, 0, 0]
        self.q_ROS_vel = [0, 0, 0, 0, 0, 0]

        self.rate = self.create_rate(10) # IMPORTANT LINE  OR ISNT IT NECCESARY???

        #Heartbeat
        #self.HEARTBEAT_PACKETS = [PacketID.POSITION, 0, 0, 0, 0, 0, 0, 0, 0, 0] # we could also request INTENSITY, VELOCITY...
        #self.DEVICE_IDS_HEARTBEAT = [0xFF]
        #self.FREQUENCY = 200
        #self.manipulator_address = (MANIPULATOR_IP_ADDRESS, MANIPULATOR_PORT)

        #self.set_fdb_heartbeat_joints()

        self.tare_ft()
    
        # Threads used to have both functions running in parallel
        threading.Thread(target = self.tx_req_and_cmd).start()
        threading.Thread(target = self.rx_receive).start()

        pass
    
    def tare_ft(self):
        tare_packet = BPLProtocol.encode_packet(0x0D, PacketID.ATI_FT_READING, BPLProtocol.encode_floats([0., 0., 0., 0., 0., 0.]))
        self.sock.sendto(tare_packet, (self.ip_address, self.port))
    '''
    def set_fdb_heartbeat_joints(self):
            JOINT_HEARTBEAT_PACKETS = [PacketID.POSITION]
            joint_heartbeat_packets_set = BPLProtocol.encode_packet(0xFF, PacketID.HEARTBEAT_SET, bytes(JOINT_HEARTBEAT_PACKETS))
            self.sock.sendto(joint_heartbeat_packets_set, self.manipulator_address)
            joint_heartbeat_frequency_set = BPLProtocol.encode_packet(0xFF, PacketID.HEARTBEAT_FREQUENCY, bytes([self.FREQUENCY]))
            self.sock.sendto(joint_heartbeat_frequency_set, self.manipulator_address)
    '''
    
    def tx_req_and_cmd(self):
        while  rclpy.ok():    
            if ((Clock().now().nanoseconds - self.last_time_req) > self.time_delay_nano_req ):  
                #REQUESTING MOTION ON JOINTS
                packets = b''
                for index, velocity in enumerate(self.q_vel):
                    device_id = index + 1
                    packets += BPLProtocol.encode_packet(device_id, PacketID.VELOCITY, BPLProtocol.encode_floats([velocity]))
                #REQUESTING FT AND JOINT STATES
                packets += BPLProtocol.encode_packet(0xFF, int(PacketID.REQUEST), [PacketID.POSITION]) 
                packets += BPLProtocol.encode_packet(13, int(PacketID.REQUEST), [PacketID.ATI_FT_READING])
                
                self.sock.sendto(packets, (self.ip_address, self.port))
                # REGISTER LAST PACKETS SENT
                self.last_time_req = Clock().now().nanoseconds
            
    #ROS subscriber callback to joint velocity commands
    def cmd_joint_position_callback(self, message):
        packet_id  = PacketID.POSITION
        i=0
        for i in self.device_ids_arm:
            if (i==0):
                device_id =7
                data = self.convert_to_bytes(message.position.a1)
            if (i==1):
                device_id =6
                data = self.convert_to_bytes(message.position.a2)
            if (i==2):
                device_id =5
                data = self.convert_to_bytes(message.position.a3)
            if (i==3):
                device_id =4
                data = self.convert_to_bytes(message.position.a4)
            if (i==4):
                device_id =3
                data = self.convert_to_bytes(message.position.a5)
            if (i==5):
                device_id =2
                data = self.convert_to_bytes(message.position.a6)
            #data = bytearray( message.value)
            encoded_packet = BPLProtocol.encode_packet(device_id, packet_id, data)
            self.sock.sendto(encoded_packet, (self.ip_address, self.port))

            
        #ROS subscriber callback to independent joint velocity commands
    def cmd_single_joint_velocity_callback(self, message):
        self.q_vel[message.device_id-1] = message.value  ## POSIBLE ERROR!!!!
        device_id  = message.device_id
        
    #ROS subscriber callback to joint velocity commands
    def cmd_local_velocity_callback(self, message):
        packet_id  = PacketID.LOCAL_VELOCITY
        data_float = []        
        data_float.append(message.linear.x)
        data_float.append(message.linear.y)
        data_float.append(message.linear.z)
        data_float.append(message.angular.x)
        data_float.append(message.angular.y)
        data_float.append(message.angular.z)
        encoded_packet = BPLProtocol.encode_packet(0x0E, packet_id, BPLProtocol.encode_floats(data_float))
        self.sock.sendto(encoded_packet, (self.ip_address, self.port))
    
    #Function to handle packets encoding
    def convert_to_bytes(self, parameter):
        if type(parameter).__name__ == 'float':
            bytes_value = bytearray(struct.pack("f", parameter))
        elif type(parameter).__name__ == 'int':
            bytes_value = bytearray(struct.pack("i", parameter))
        elif type(parameter).__name__ == 'list':
            bytes_value= bytearray()
            for val in parameter:
                bytes_value +=bytearray(struct.pack("f", val))
        else:
            return parameter
        return bytes_value


    # Handles the packets received from the Bravo 7 and publish ROS topics
    def rx_receive(self):
        packet_reader = PacketReader()
        while rclpy.ok():
            if ((Clock().now().nanoseconds - self.time_receiver) > self.time_delay_nano_recv ):
                    self.time_receiver = Clock().now().nanoseconds
                    try:
                        data, address = self.sock.recvfrom(4096)
                        data = bytearray(data)
                    except socket.error as e:
                        continue
                    if data:                
                        packets = packet_reader.receive_bytes(data)

                        for packet in packets:
                            device_id = packet[0]
                            packet_id = packet[1]
                            data = packet[2]

                            ros_packet = Packet()
                            ros_packet.device_id = device_id
                            ros_packet.packet_id = packet_id
                            ros_packet.data = list(data)
                            
                            # Classification of packet to publish into different ROS topics
                            if (packet_id == PacketID.POSITION):
                                #print("position recieving")
                                self.robot_joint_positions[device_id-1] = BPLProtocol.decode_floats(data)[0]
                                self.q_joints_received[device_id-1] = True
                                if (self.q_joints_received[0] and self.q_joints_received[1] and self.q_joints_received[2] and self.q_joints_received[3] and self.q_joints_received[4] and self.q_joints_received[5]):        
                                    for i in range(self.arm_joint_dof):
                                        #The device_id takes the values 0, 1, 2, 3, 4, 5
                                        self.arm_joint_state_msg.position[i] = (self.robot_joint_positions[5-i])                        
                                        if (i==0):
                                            self.robot_joint_pos.position.a1 = self.robot_joint_positions[6-i]
                                        if (i==1):
                                            self.robot_joint_pos.position.a2 = self.robot_joint_positions[6-i]
                                        if (i==2):
                                            self.robot_joint_pos.position.a3 = self.robot_joint_positions[6-i]
                                        if (i==3):
                                            self.robot_joint_pos.position.a4 = self.robot_joint_positions[6-i]
                                        if (i==4):
                                            self.robot_joint_pos.position.a5 = self.robot_joint_positions[6-i]
                                        if (i==5):
                                            self.robot_joint_pos.position.a6 = self.robot_joint_positions[6-i]

                                    self.robot_joint_pos.stamp = Clock().now().to_msg()                                  
                                    self.robot_joint_pos_pub.publish(self.robot_joint_pos)

                                    self.arm_joint_state_msg.header.stamp = Clock().now().to_msg()
                                    self.robot_joint_state_pub.publish(self.arm_joint_state_msg)

                                    self.q_joints_received[0] = False; self.q_joints_received[1] = False; self.q_joints_received[2] = False;
                                    self.q_joints_received[3] = False; self.q_joints_received[4] = False; self.q_joints_received[5] = False;
                                            
                            elif packet_id == PacketID.ATI_FT_READING:
                                #print("wrench recieving")
                                
                                self.wrench_msg.header.stamp = Clock().now().to_msg()
                                ft_readings = BPLProtocol.decode_floats(data)
                                self.wrench_msg.wrench.force.x = ft_readings[0]
                                self.wrench_msg.wrench.force.y = ft_readings[1]
                                self.wrench_msg.wrench.force.z = ft_readings[2]
                                self.wrench_msg.wrench.torque.x = ft_readings[3]
                                self.wrench_msg.wrench.torque.y = ft_readings[4]
                                self.wrench_msg.wrench.torque.z = ft_readings[5]
                                self.wrench_pub.publish(self.wrench_msg)

                            else:
                                #print("cucu recieving")
                                self.other_message.stamp = Clock().now().to_msg()
                                self.other_message.device_id = device_id
                                self.other_message.packet_id = packet_id
                                data_float = []                    
                                for data_int in data:
                                    data_float.append(float(data_int))
                                self.other_message.data = data_float
                                self.other_pub.publish(self.other_message)

                    pass
def main(args=None):
    rclpy.init(args=args)
    jsp = udp_bravo7_bringup_unite_v6()
    rclpy.spin(jsp)


if __name__ == "__main__":
    main()
