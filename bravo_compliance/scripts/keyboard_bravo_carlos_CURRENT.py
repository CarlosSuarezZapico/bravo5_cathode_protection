#!/usr/bin/env python3
import rclpy
from bpl_msgs.msg import SingleFloat
from pynput.keyboard import Key
from pynput.keyboard import Listener

global cmd_velocity_pub

def main():
    global cmd_velocity_pub
    rclpy.init()
    node = rclpy.create_node("my_node")
    cmd_velocity_pub = node.create_publisher(SingleFloat, 'b7m_0/cmd_current',  10)
    keyboard_b7m_main()


def keyboard_press(key):
    global cmd_velocity_pub
    cmd_current_message = SingleFloat()
    try:
        key_pressed = key.char
    except:
        return
    if key.char == 'q':        
        cmd_current_message.device_id = 7
        cmd_current_message.value = 500.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('q pressed')
    elif key.char == 'a':        
        cmd_current_message.device_id = 7
        cmd_current_message.value = -500.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('a pressed')
    elif key.char == 'w':        
        cmd_current_message.device_id = 6
        cmd_current_message.value = 500.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('w pressed')
    elif key.char == 's':        
        cmd_current_message.device_id = 6
        cmd_current_message.value = -500.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('s pressed')
    elif key.char == 'e':        
        cmd_current_message.device_id = 5
        cmd_current_message.value = 500.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('e pressed')
    elif key.char == 'd':        
        cmd_current_message.device_id = 5
        cmd_current_message.value = -500.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('d pressed')
    elif key.char == 'r':        
        cmd_current_message.device_id = 4
        cmd_current_message.value = 500.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('r pressed')
    elif key.char == 'f':        
        cmd_current_message.device_id = 4
        cmd_current_message.value = -500.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('f pressed')
    elif key.char == 't':        
        cmd_current_message.device_id = 3
        cmd_current_message.value = 500.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('t pressed')
    elif key.char == 'g':        
        cmd_current_message.device_id = 3
        cmd_current_message.value = -500.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('g pressed')   
    elif key.char == 'y':        
        cmd_current_message.device_id = 2
        cmd_current_message.value = 500.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('y pressed')
    elif key.char == 'h':        
        cmd_current_message.device_id = 2
        cmd_current_message.value = -500.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('h pressed')   
    elif key.char == 'u':        
        cmd_current_message.device_id = 1
        cmd_current_message.value = 0.0 #prev 4
        cmd_velocity_pub.publish(cmd_current_message)
        print('u pressed')
    elif key.char == 'j':        
        cmd_current_message.device_id = 1
        cmd_current_message.value = -0.0 #prev 4
        cmd_velocity_pub.publish(cmd_current_message)
        print('j pressed')   
    elif key.char == 'k':        
        exit()
        print('k pressed') 
    else:
         print("COMMAND NOT RECOGNIZE")

def keyboard_release(key): 
    global cmd_velocity_pub   
    cmd_current_message = SingleFloat()
    if key == Key.esc:
        return False
    try:
        key_released = key.char
    except:
        return
    if key.char == 'q':        
        cmd_current_message.device_id = 7
        cmd_current_message.value = 0.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('q pressed')
    elif key.char == 'a':        
        cmd_current_message.device_id = 7
        cmd_current_message.value = 0.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('a pressed')
    elif key.char == 'w':        
        cmd_current_message.device_id = 6
        cmd_current_message.value = 0.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('w pressed')
    elif key.char == 's':        
        cmd_current_message.device_id = 6
        cmd_current_message.value = 0.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('s pressed')
    elif key.char == 'e':        
        cmd_current_message.device_id = 5
        cmd_current_message.value = 0.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('e pressed')
    elif key.char == 'd':        
        cmd_current_message.device_id = 5
        cmd_current_message.value = 0.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('d pressed')   
    elif key.char == 'r':        
        cmd_current_message.device_id = 4
        cmd_current_message.value = 0.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('r pressed')
    elif key.char == 'f':        
        cmd_current_message.device_id = 4
        cmd_current_message.value = 0.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('f pressed')   
    elif key.char == 't':        
        cmd_current_message.device_id = 3
        cmd_current_message.value = 0.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('t pressed')
    elif key.char == 'g':        
        cmd_current_message.device_id = 3
        cmd_current_message.value = 0.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('g pressed')     
    elif key.char == 'y':        
        cmd_current_message.device_id = 2
        cmd_current_message.value = 0.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('y pressed')
    elif key.char == 'h':        
        cmd_current_message.device_id = 2
        cmd_current_message.value = 0.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('h pressed')
    elif key.char == 'u':        
        cmd_current_message.device_id = 1
        cmd_current_message.value = 0.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('u pressed')
    elif key.char == 'j':        
        cmd_current_message.device_id = 1
        cmd_current_message.value = 0.0
        cmd_velocity_pub.publish(cmd_current_message)
        print('j pressed')     
    else:
        print("COMMAND NOT RECOGNIZE")



def keyboard_b7m_main():

    if rclpy.ok():
        global cmd_velocity_pub 
        with Listener(
                    on_press=keyboard_press,
                    on_release=keyboard_release
                    ) as listener:
            listener.join()

    
if __name__ == "__main__":
    main()

