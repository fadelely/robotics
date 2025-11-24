## Running the robot
You can run the robot using:
```bash
cd robot
colcon build --packages-select mujoco_ros2
source install/setup.bash
ros2 run mujoco_ros2 mujoco_node src/mujoco_ros2/model/scene.xml 
```
Or simply run the `run.sh` located in the `robot` directory.
## Keybinds

| Key       | Action                                 |
|-----------|----------------------------------------|
| `D`       | Delete the trajectory path drawn       |
| `H`       | Hide/Show the drawn trajectory path    |
| `T`       | Toggle drawing of the trajectory path  |
| `1`       | Move box to the middle of the conveyor |
| `2`       |Move box to end of conveyor             |



[Demo Link](https://drive.google.com/file/d/1eo6mbCz05U3W2a4jaB6e1DSiGz3171jw/view?usp=drivesdk)
