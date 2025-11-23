### 0. 환경세팅
```
여기에 준호 오빠 패키지 파일 병합함
아래 패키지는 추후에 삭제해야 해(주노 패키지에 없는 패키지야)

/home/jinsun/jetank_ws/src/jetank_ros2_main
/home/jinsun/jetank_ws/src/jetank_perception
/home/jinsun/jetank_ws/src/jetank_simulation
/home/jinsun/jetank_ws/src/jetank_motor_control
```

<br><br>

# 0. 설치
```
sudo apt install ros-humble-rqt* -y
rqt --force-discover
```
```
sudo apt update
sudo apt install ros-humble-ign-ros2-control ros-humble-ign-ros2-control-demos
sudo apt install ros-humble-gz-ros2-control ros-humble-gz-ros2-control-demos
source /opt/ros/humble/setup.bash
```

<br><br>

# 1. 빌드
```
cd ~/jetank_ws
colcon build
source ~/jetank_ws/install/setup.bash
```

<br><br>

# 2. 실행
```
sudo pkill -9 -f "rqt|gazebo|rviz|ign"
sudo pkill -9 -f "ros2|gz|gazebo|ign|rqt"
```
```
cd ~/jetank_ws
source ~/jetank_ws/install/setup.bash
ros2 launch robot_gz_startup spawn_robot.launch.py robot_name:=jetank ns:=robot
```    
```
/robot/diff_drive_controller/cmd_vel_unstamped
```
```
cd ~/jetank_ws
source ~/jetank_ws/install/setup.bash
ros2 run rviz2 rviz2 --ros-args -r /tf:=/robot/tf -r /tf_static:=/robot/tf_static
```

<br><br>

# 3. 확인
<mark>그리퍼가 우측으로 돌아가 있으면 안 돼!!! <br>
<img width="2880" height="1800" alt="image" src="https://github.com/user-attachments/assets/8f3746d5-727d-436b-924b-e67fedead9cd" /><br>

<mark>실행했을 때, 이렇게 보여야 해 <br>
<img width="2880" height="1800" alt="image" src="https://github.com/user-attachments/assets/b601ea5d-3a5b-42db-a5ca-384a30901b2f" /><br>

<mark>컨트롤러도 이렇게 클릭해서 변경이 가능해야 해 <br>
<img width="2880" height="1800" alt="image" src="https://github.com/user-attachments/assets/4ca18151-67ff-4f6a-a035-3fe593b3848a" /><br>

<mark>rviz 상에서도 잘 보임 <br>
<img width="2880" height="1800" alt="image" src="https://github.com/user-attachments/assets/b09554b5-cd3d-447f-86a8-e45c2267c96e" /><br>
<img width="2880" height="1800" alt="image" src="https://github.com/user-attachments/assets/333cddda-9175-438c-af0e-03c9133ff324" /><br>
