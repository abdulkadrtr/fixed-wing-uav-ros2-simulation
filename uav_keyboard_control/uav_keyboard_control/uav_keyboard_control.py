import rclpy
from rclpy.node import Node
from uav_control_msgs.msg import UavControlMessage
import os
from pynput import keyboard
import threading
import time

pressed_keys = set()

def on_press(key):
    try:
        if hasattr(key, 'char'): 
            pressed_keys.add(key.char)
        else:
            pressed_keys.add(key)
    except AttributeError:
        pressed_keys.add(key)

def on_release(key):
    try:
        if hasattr(key, 'char'): 
            pressed_keys.remove(key.char)
        else:
            pressed_keys.remove(key)  
    except KeyError:
        try:
            pressed_keys.remove(key)
        except KeyError:
            pass
    if key == keyboard.Key.esc:
        return False

def control():
    global roll, pitch, yaw, speed, flap, brakesTorque
    roll_,pitch_,yaw_,speed_ = 0.0,0.0,0.0,0.0
    yaw_increment = 0.18
    roll_increment = 0.18
    pitch_increment = 0.18
    speed_increment = 5
    while True:
        flag = 0
        if 'w' in pressed_keys:
            pitch_ += pitch_increment
            flag = 1
        if 'a' in pressed_keys:
            roll_ -= roll_increment
            flag = 1
        if 's' in pressed_keys:
            pitch_ -= pitch_increment
            flag = 1
        if 'd' in pressed_keys:
            roll_ += roll_increment
            flag = 1
        if 'q' in pressed_keys:
            yaw_ += yaw_increment
            flag = 1
        if 'e' in pressed_keys:
            yaw_ -= yaw_increment
            flag = 1
        if 'f' in pressed_keys:
            flap = 1 if flap == 0 else 0
        if 'b' in pressed_keys:
            brakesTorque = 1 if brakesTorque == 0 else 0
        if 't' in pressed_keys:
            speed_ += speed_increment
        if 'g' in pressed_keys:
            speed_ -= speed_increment
        if flag == 0:
            yaw_ = 0.0
            roll_ = 0.0
            pitch_ = 0.0
        clear_screen()
        print(message)
        yaw = clamp(yaw_, -1.0, 1.0)
        roll = clamp(roll_, -1.0, 1.0)
        pitch = clamp(pitch_, -1.0, 1.0)
        speed = clamp(speed_, 0.0, 100.0)
        time.sleep(0.1)

def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')

def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

roll,pitch,yaw,speed,flap,brakesTorque = 0.0,0.0,0.0,0.0,0,0

message = """
    FIXED WING UAV CONTROL NODE

PITCH        : W / S
ROLL         : A / D
YAW          : Q / E
TARGET SPEED : T / G
FLAP         : F
BRAKES TORQUE: B 

"""

class UAVControlNode(Node):
    def __init__(self):
        super().__init__('uav_control_node')
        self.publisher = self.create_publisher(UavControlMessage, '/uav_control', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.start_listeners()
        print(message)



    def timer_callback(self):
        global roll, pitch, yaw, speed, flap, brakesTorque
        msg = UavControlMessage()
        msg.roll = roll
        msg.pitch = pitch
        msg.yaw = yaw
        msg.target_speed = speed
        msg.flap = flap
        msg.brakes_torque = brakesTorque
        self.publisher.publish(msg)

    def start_listeners(self):
        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()
        control_thread = threading.Thread(target=control)
        control_thread.start()
        listener_thread = threading.Thread(target=listener.join)
        listener_thread.start()    
        control_thread = threading.Thread(target=control_thread.join)
        control_thread.start()




def main(args=None):
    rclpy.init(args=args)
    uav_control_node = UAVControlNode()
    rclpy.spin(uav_control_node)
    uav_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
