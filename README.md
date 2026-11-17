#빌드

```
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```
#실행
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
#Localization 켜기
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run localization MyProject
```

