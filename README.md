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
<img width="2880" height="1800" alt="Image" src="https://private-user-images.githubusercontent.com/68187536/516251016-1db20ce5-9ff1-4887-a752-c564c6209371.png?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NjM2MjExNTEsIm5iZiI6MTc2MzYyMDg1MSwicGF0aCI6Ii82ODE4NzUzNi81MTYyNTEwMTYtMWRiMjBjZTUtOWZmMS00ODg3LWE3NTItYzU2NGM2MjA5MzcxLnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTExMjAlMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUxMTIwVDA2NDA1MVomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTY1MmE4N2RkZTYxNTA0MGQ3ZGQzMDI4MTBkMzk3Y2M4YWUwZGNiYWY5NDhkYjA2ZTAxMDk2YjMwOTg4NDQzMzAmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.Z2aEFzrElmBnzHWorh1Ofq1rivtZnBWQ87onNfWBgBY" /> <br>

<mark>실행했을 때, 이렇게 보여야 해 <br>
<img width="2880" height="1800" alt="Image" src="https://private-user-images.githubusercontent.com/68187536/516638714-47de7e08-8f0f-448c-a423-ecf711e215d4.png?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NjM2MjExOTUsIm5iZiI6MTc2MzYyMDg5NSwicGF0aCI6Ii82ODE4NzUzNi81MTY2Mzg3MTQtNDdkZTdlMDgtOGYwZi00NDhjLWE0MjMtZWNmNzExZTIxNWQ0LnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTExMjAlMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUxMTIwVDA2NDEzNVomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPWJhMWU0ZjdhNGQxMzgwYWNkNjVjOTk1NTBiNzQzMDI5ZTAxNDk3MjMxODI2OTljZWQ0YThmYTg3Mjg2ZmQyZTYmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.djBf9mVgioZpxX-203M7nspvHrVs1S_iCSmp7j0GJr4" /> <br>

<mark>컨트롤러도 이렇게 클릭해서 변경이 가능해야 해 <br>
<img width="2880" height="1800" alt="Image" src="https://private-user-images.githubusercontent.com/68187536/516639074-53daf5c7-0b32-4c3e-815a-5d467979e756.png?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NjM2MjExOTcsIm5iZiI6MTc2MzYyMDg5NywicGF0aCI6Ii82ODE4NzUzNi81MTY2MzkwNzQtNTNkYWY1YzctMGIzMi00YzNlLTgxNWEtNWQ0Njc5NzllNzU2LnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTExMjAlMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUxMTIwVDA2NDEzN1omWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPWE5MTg0NzJlYjMzOGJhNTUwZGNmYTJlMTE1YjY0YmI2YmQwNTNmMjJmOTVmNzY2YThiYTRlZTI1YjliYTA1YTcmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.bJBGs8gIhYvIy_AJxREuStGgSa1Ft4XACUtVY88uxKw" /> <br>

<mark>rviz 상에서도 잘 보임 <br>
<img width="2880" height="1800" alt="Image" src="https://private-user-images.githubusercontent.com/68187536/516640078-ebff8ad2-cc53-4b62-9b11-67bb1e775099.png?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NjM2MjEyNzQsIm5iZiI6MTc2MzYyMDk3NCwicGF0aCI6Ii82ODE4NzUzNi81MTY2NDAwNzgtZWJmZjhhZDItY2M1My00YjYyLTliMTEtNjdiYjFlNzc1MDk5LnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTExMjAlMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUxMTIwVDA2NDI1NFomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPWM3M2I3N2RkY2U3ZTBkZGI0OGY0ZjhkZTBlMTcyZGU1ZTFiMmYzYjJhNTc1MzAyY2Y0ZTU1Y2JjNDRlMWQ1NDQmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.ix7c1pbQGVomGO9T2DETT-MFj6Ph1BFE_02anDP6zIE" /> <br>

<img width="2880" height="1800" alt="Image" src="https://private-user-images.githubusercontent.com/68187536/516640146-764de213-15dc-422a-a316-cb3b8715b84c.png?jwt=eyJ0eXAiOiJKV1QiLCJhbGciOiJIUzI1NiJ9.eyJpc3MiOiJnaXRodWIuY29tIiwiYXVkIjoicmF3LmdpdGh1YnVzZXJjb250ZW50LmNvbSIsImtleSI6ImtleTUiLCJleHAiOjE3NjM2MjEyNzQsIm5iZiI6MTc2MzYyMDk3NCwicGF0aCI6Ii82ODE4NzUzNi81MTY2NDAxNDYtNzY0ZGUyMTMtMTVkYy00MjJhLWEzMTYtY2IzYjg3MTViODRjLnBuZz9YLUFtei1BbGdvcml0aG09QVdTNC1ITUFDLVNIQTI1NiZYLUFtei1DcmVkZW50aWFsPUFLSUFWQ09EWUxTQTUzUFFLNFpBJTJGMjAyNTExMjAlMkZ1cy1lYXN0LTElMkZzMyUyRmF3czRfcmVxdWVzdCZYLUFtei1EYXRlPTIwMjUxMTIwVDA2NDI1NFomWC1BbXotRXhwaXJlcz0zMDAmWC1BbXotU2lnbmF0dXJlPTQzY2FkMGQwYzQ5NTUwYWJlMmMyZDAxMzU3MGUzMGI0OGViYmFjNWY5MGViMTczMTk0YzA3NDgwM2VlZmM4ZTYmWC1BbXotU2lnbmVkSGVhZGVycz1ob3N0In0.ybL1QjObqRI_rkSMbISrSlmlMqs7OSCB6XUrQqAZXFQ" /> <br>