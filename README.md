## requirment

```
#패키지 이름 
robot_vision
humanoid_interface
insta360_usb_cam
Localization
ik_walk
dynamixel_hardware_interface
ebimu
```

## 빌드

```
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 실행
```
# insta 카메라 켜기
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
ros2 launch insta360_usb_cam usb_cam.launch.py
```
```
# vision package 켜기
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
ros2 launch robot_vision robot_vision.launch.py
```

```
# dynamixel 켜기 
cd ~/colcon_ws
colcon build --packages-select dynamixel_hardware_interface
source install/setup.bash
ros2 launch dynamixel_hardware_interface dynamixel_hardware.launch.py
```

```
# ik_walk 켜기
cd ~/colcon_ws
colcon build --packages-select ik_walk
source install/setup.bash
ros2 run ik_walk ik_walk
```

```
# imu 켜기
cd ~/colcon_ws
colcon build --packages-select ebimu
source install/setup.bash
ros2 launch ebimu e2box_imu_9dofv4.launch.py
```

```
#Localization 켜기
cd ~/colcon_ws
colcon build --packages-select localization
source install/setup.bash
ros2 launch localization localization.launch.py
```

