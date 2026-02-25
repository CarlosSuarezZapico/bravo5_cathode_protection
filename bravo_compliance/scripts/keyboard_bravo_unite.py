#!/usr/bin/env python
"""
	  @author   Carlos Suarez Zapico carlossuarezzapico@gmail.com
 
	  @internal
	  Created  5-Oct-2021
	  Modified for ROS2 5-Dec-2023
	  Unite Project
"""
import rclpy
from bpl_msgs.msg import SingleFloat
from pynput.keyboard import Key
from pynput.keyboard import Listener


cmd_velocity_pub = rclpy.create_publisher(SingleFloat,'b7m_0/cmd_velocity', 10)
cmd_velocity_message = SingleFloat()

def keyboard_press(key):
    try:
        key_pressed = key.char
    except:
        return
    if key.char == 'q':        
        cmd_velocity_message.device_id = 7
        cmd_velocity_message.value = 0.2
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('q pressed')
    elif key.char == 'a':        
        cmd_velocity_message.device_id = 7
        cmd_velocity_message.value = -0.2
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('a pressed')
    elif key.char == 'w':        
        cmd_velocity_message.device_id = 6
        cmd_velocity_message.value = 0.5
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('w pressed')
    elif key.char == 's':        
        cmd_velocity_message.device_id = 6
        cmd_velocity_message.value = -0.5
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('s pressed')
    elif key.char == 'e':        
        cmd_velocity_message.device_id = 5
        cmd_velocity_message.value = 0.5
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('e pressed')
    elif key.char == 'd':        
        cmd_velocity_message.device_id = 5
        cmd_velocity_message.value = -0.5
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('d pressed')
    elif key.char == 'r':        
        cmd_velocity_message.device_id = 4
        cmd_velocity_message.value = 0.5
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('r pressed')
    elif key.char == 'f':        
        cmd_velocity_message.device_id = 4
        cmd_velocity_message.value = -0.5
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('f pressed')
    elif key.char == 't':        
        cmd_velocity_message.device_id = 3
        cmd_velocity_message.value = 10.0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('t pressed')
    elif key.char == 'g':        
        cmd_velocity_message.device_id = 3
        cmd_velocity_message.value = -10.0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('g pressed')   
    elif key.char == 'y':        
        cmd_velocity_message.device_id = 2
        cmd_velocity_message.value = 1.0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('y pressed')
    elif key.char == 'h':        
        cmd_velocity_message.device_id = 2
        cmd_velocity_message.value = -1.0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('h pressed')   
    elif key.char == 'u':        
        cmd_velocity_message.device_id = 1
        cmd_velocity_message.value = 4
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('u pressed')
    elif key.char == 'j':        
        cmd_velocity_message.device_id = 1
        cmd_velocity_message.value = -4
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('j pressed')   
    elif key.char == 'k':        
        exit()
        print('k pressed') 

def keyboard_release(key):    
    if key == Key.esc:
        return False
    try:
        key_released = key.char
    except:
        return
    if key.char == 'q':        
        cmd_velocity_message.device_id = 7
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('q pressed')
    elif key.char == 'a':        
        cmd_velocity_message.device_id = 7
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('a pressed')
    elif key.char == 'w':        
        cmd_velocity_message.device_id = 6
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('w pressed')
    elif key.char == 's':        
        cmd_velocity_message.device_id = 6
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('s pressed')
    elif key.char == 'e':        
        cmd_velocity_message.device_id = 5
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('e pressed')
    elif key.char == 'd':        
        cmd_velocity_message.device_id = 5
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('d pressed')   
    elif key.char == 'r':        
        cmd_velocity_message.device_id = 4
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('r pressed')
    elif key.char == 'f':        
        cmd_velocity_message.device_id = 4
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('f pressed')   
    elif key.char == 't':        
        cmd_velocity_message.device_id = 3
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('t pressed')
    elif key.char == 'g':        
        cmd_velocity_message.device_id = 3
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('g pressed')     
    elif key.char == 'y':        
        cmd_velocity_message.device_id = 2
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('y pressed')
    elif key.char == 'h':        
        cmd_velocity_message.device_id = 2
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('h pressed')
    elif key.char == 'u':        
        cmd_velocity_message.device_id = 1
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('u pressed')
    elif key.char == 'j':        
        cmd_velocity_message.device_id = 1
        cmd_velocity_message.value = 0
        cmd_velocity_pub.publish(cmd_velocity_message)
        print('j pressed')     


def keyboard_b7m_main():

    if not rclpy.is_shutdown():
        with Listener(
                    on_press=keyboard_press,
                    on_release=keyboard_release
                    ) as listener:
            listener.join()


if __name__ == "__main__":
    rclpy.init("keyboard_b7m")
    keyboard_b7m_main()
