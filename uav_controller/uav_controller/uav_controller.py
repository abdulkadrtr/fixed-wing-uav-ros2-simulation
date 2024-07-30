import rclpy
from rclpy.node import Node
from uav_info_msgs.msg import UavInfoMessage
from uav_control_msgs.msg import UavControlMessage
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import socket
import time
import threading
import numpy as np

host = '127.0.0.1'
port_send = 14000
port_recv = 14001
roll,pitch,yaw,speed,flap,brakesTorque = 0.0,0.0,0.0,0,0,0
data_send_str = str(roll) + "," + str(pitch) + "," + str(yaw) + "," + str(speed) + "," + str(flap) + "," + str(brakesTorque)
x_pose, y_pose, altitude, euler_x, euler_y, euler_z, speed_info = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0

def send_data():
    global data_send_str
    while True:
        try:
            client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            client_socket.connect((host, port_send))
            print("[INFO] Control connection is established")
            while True:
                try:
                    client_socket.send(data_send_str.encode('utf-8'))
                    time.sleep(0.1)
                except Exception as e:
                    print(e)
                    break
        except Exception as e:
            pass



def get_data():
    global x_pose, y_pose, altitude, euler_x, euler_y, euler_z, speed_info
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((host, port_recv))
    server.listen(1)
    # print("Server is listening")
    while True:
        try:
            client_socket, addr = server.accept()
            print("[INFO] Data connection is established")
            while True:
                data = client_socket.recv(1024).decode('utf-8')
                if not data:
                    break
                values = data.split(",")
                x_pose = float(values[0])
                altitude = float(values[1])
                y_pose = float(values[2])
                euler_x = float(values[3])    
                euler_y = float(values[4])
                euler_z = float(values[5])
                speed_info = float(values[6])
                #print("x_konum : {:.3f} y_konum : {:.3f} z_konum : {:.3f} hız : {:.3f} x_rot : {:.3f} y_rot : {:.3f} z_rot : {:.3f}".format(x_loc, y_loc, z_loc, speed, x_rot, y_rot, z_rot))
        except Exception as e:
            # print("veri alma hatası {e}")
            pass
        finally:
            client_socket.close()
            # print("Client is disconnected")


def euler_to_quaternion(euler):
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    q = np.zeros(4)
    q[0] = cy * cp * cr + sy * sp * sr
    q[1] = cy * cp * sr - sy * sp * cr
    q[2] = sy * cp * sr + cy * sp * cr
    q[3] = sy * cp * cr - cy * sp * sr
    return q

class UAVController(Node):
    def __init__(self):
        super().__init__('uav_controller')
        self.subscription = self.create_subscription(UavControlMessage,'/uav_control',self.control_callback,10)
        self.publisher = self.create_publisher(UavInfoMessage, '/uav_info', 10)
        self.path_publisher = self.create_publisher(Path, '/uav_path', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.path = []
        self.path_msg = Path()
        threading.Thread(target=send_data).start()
        threading.Thread(target=get_data).start()

    def timer_callback(self):
        global x_pose, y_pose, altitude, euler_x, euler_y, euler_z, speed_info
        msg = UavInfoMessage()
        msg.x_pose = x_pose
        msg.y_pose = y_pose
        msg.altitude = altitude
        msg.euler_x = euler_x
        msg.euler_y = euler_y
        msg.euler_z = euler_z
        msg.speed = speed_info
        self.publisher.publish(msg)
        #for path visualization
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = x_pose
        pose_stamped.pose.position.y = y_pose
        pose_stamped.pose.position.z = altitude
        euler_angles_rad = np.deg2rad([euler_x, euler_y, euler_z])
        q = euler_to_quaternion(euler_angles_rad)
        pose_stamped.pose.orientation.x = q[0]
        pose_stamped.pose.orientation.y = q[1]
        pose_stamped.pose.orientation.z = q[2]
        pose_stamped.pose.orientation.w = q[3]
        self.path.append(pose_stamped)
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.header.frame_id = "map"
        self.path_msg.poses = self.path
        self.path_publisher.publish(self.path_msg)

    def control_callback(self, msg):
        global data_send_str
        roll = msg.roll
        pitch = msg.pitch
        yaw = msg.yaw
        target_speed = msg.target_speed
        flap = msg.flap
        brakes_torque = msg.brakes_torque
        data_send_str = str(roll) + "," + str(pitch) + "," + str(yaw) + "," + str(target_speed) + "," + str(flap) + "," + str(brakes_torque)


def main(args=None):
    rclpy.init(args=args)
    uav_controller = UAVController()
    rclpy.spin(uav_controller)
    uav_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
