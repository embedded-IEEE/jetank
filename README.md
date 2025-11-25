### Acknowledgements
I would like to express my sincere appreciation to the open-source community and the developers whose work made this project possible.  
Their contributions, documentation, and shared models greatly supported the development and integration of this project.

Referenced repositories and resources:
- https://github.com/kvgork/jetank_ROS  
- https://github.com/decipher2k/Waveshare-Jetank-ROS2
- https://github.com/ai-practice-enterprice/jetank  
- https://github.com/ai-practice-enterprice/robot_gz_startup

<br><br>

# 0. 설치
아래 명령어 실행 후 팝업창이 실행되면 정상 동작한 것이니 종료하면 돼
```
sudo apt update
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-rqt* -y
rqt --force-discover
```
```
sudo apt update
source /opt/ros/humble/setup.bash
sudo apt install ros-humble-ros-gz -y
sudo apt install ros-humble-gz-ros2-control ros-humble-gz-ros2-control-demos -y
sudo apt install ros-humble-ign-ros2-control ros-humble-ign-ros2-control-demos -y
```
```
cd
git clone https://github.com/Jinsun-Lee/jetank_ws.git
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
아래 명령어 실행으로 제대로 종료되지 않은 프로그램 제대로 종료
```
sudo pkill -9 -f "rqt|gazebo|rviz|ign"
sudo pkill -9 -f "ros2|gz|gazebo|ign|rqt"
```

<br>

아래 명령어 실행으로 로봇 시뮬레이션을 실행
```
cd ~/jetank_ws
source ~/jetank_ws/install/setup.bash
ros2 launch robot_gz_startup spawn_robot.launch.py robot_name:=jetank ns:=robot
```

<br>

`rqt_robot_steering_RobotSteering - rqt` 팝업창에서 `/cmd_vel`을 아래 내용으로 수정
```
/robot/diff_drive_controller/cmd_vel_unstamped
```

<br>

아래 명령어 실행으로 rviz를 실행
```
cd ~/jetank_ws
source ~/jetank_ws/install/setup.bash
ros2 run rviz2 rviz2 --ros-args -r /tf:=/robot/tf -r /tf_static:=/robot/tf_static
```

<br><br>

# 3. 확인
<mark>그리퍼가 우측으로 돌아가 있으면 안 돼!!!</mark> <br>
<img width="2880" height="1800" alt="image" src="https://github.com/user-attachments/assets/8f3746d5-727d-436b-924b-e67fedead9cd" /><br>

<mark>실행했을 때, 이렇게 보여야 해</mark> <br>
<img width="2880" height="1800" alt="image" src="https://github.com/user-attachments/assets/b601ea5d-3a5b-42db-a5ca-384a30901b2f" /><br>

<mark>컨트롤러도 이렇게 클릭해서 변경이 가능해야 해</mark> <br>
<img width="2880" height="1800" alt="image" src="https://github.com/user-attachments/assets/4ca18151-67ff-4f6a-a035-3fe593b3848a" /><br>

<mark>rviz 상에서도 잘 보여(사진처럼 세팅해!)</mark> <br>
<img width="2880" height="1800" alt="image" src="https://github.com/user-attachments/assets/b09554b5-cd3d-447f-86a8-e45c2267c96e" /><br>
<img width="2880" height="1800" alt="image" src="https://github.com/user-attachments/assets/333cddda-9175-438c-af0e-03c9133ff324" /><br>
