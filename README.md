to run:
```bash
cd robot
colcon build --packages-select mujoco_ros2
source install/setup.bash
ros2 run mujoco_ros2 mujoco_node src/mujoco_ros2/model/scene.xml 
```

