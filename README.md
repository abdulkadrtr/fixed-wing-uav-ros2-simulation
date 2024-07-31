# Fixed Wing UAV ROS2 Simulation

![fixed-wing-uav](https://github.com/user-attachments/assets/7080bc56-47d4-468c-bce7-c97f91d537e3)

## Demo 

https://youtu.be/OdzIkL9VDmY?si=tY7iI35HB6NyqarO

## 
This project is a simulation that uses Unity [aircraft model physics](https://github.com/gasgiant/Aircraft-Physics) and can be controlled via a ROS2 interface. A bridge has been configured between Unity and ROS2, allowing for the following control commands to be sent to the aircraft:

- `roll`
- `pitch`
- `yaw`
- `target_speed`: Desired speed of the aircraft. (The PID algorithm within the simulation attempts to match the aircraft's current speed to this value, adhering to the physical limits of the aircraft.)
- `flap`: Used to move the aircraft's flaps, 0 or 1.
- `brakes_torque`: The braking status of the aircraft's wheels, 0 or 1.

The `uav_control_msgs` package has been created for sending these control commands. The message type is defined as follows:

```plaintext
float32 roll
float32 pitch
float32 yaw
float32 target_speed
int32 flap
int32 brakes_torque
````

## Data Received from the Aircraft
The data received from the aircraft includes:

- Aircraft x, y position and altitude (height)
- Aircraft orientation in Euler angles (x, y, z)
- Instantaneous speed of the aircraft

This information is defined in the uav_info_msgs package as follows:
```
float32 x_pose
float32 y_pose
float32 altitude
float32 euler_x
float32 euler_y
float32 euler_z
float32 speed
```

## Usage
When the uav_controller package is run, the bridge between ROS2 and Unity is initiated, and the `/uav_control` and `/uav_info` topics start operating. Commands can be issued to the UAV by sending a `UavControlMessage` type message to the `/uav_control` topic. Information about the UAV can be received from the `/uav_info` topic.

The `uav_keyboard_control` package is designed for controlling the aircraft using keyboard inputs. This package publishes keyboard controls to the `/uav_control` topic, allowing the aircraft to be driven via keyboard commands.

## Setup Instructions
Ensure you have `ROS2 Humble` installed on your Ubuntu system and open the `ros2_ws` directory to clone this repository. Then, follow these steps:

1 - [Download](https://drive.google.com/drive/folders/1Vs01aubhgjO15QiIdTZjCxT2dU8q7iDB?usp=sharing) the simulation environment and start the `aircraft.x86_64` application. This will open the simulation environment.



2 - Run the following command to establish the bridge between ROS2 and Unity, and activate the control and information topics:

`ros2 launch uav_controller robot_state_publisher.launch.py`

This command will also open an RVIZ2 screen, visualizing the path taken by the aircraft in 3D.

3 - To control the UAV using the keyboard, run the keyboard control package:

`ros2 run uav_keyboard_control uav_keyboard_control`

## More

To reset the simulation environment, you can use the `ctrl + c` key combination while on the simulation screen.
