ROS 2 Humble, Gazebo Harmonic, PX4 SITL 환경에서 창고 월드 기반 드론 오프보드 미션을 실행하기 위한 패키지입니다.
자연어 명령을 통해 드론이 창고 내 지정된 웨이포인트로 이동하고, Aruco 마커를 인식하며 정밀 착륙하는 시스템입니다.
<br>

---
<br>

## Dependencies

* __[Python](https://www.python.org/)__ 3.10 or compatible
* __[ROS 2 Humble](https://docs.ros.org/en/humble/)__ with __[cv_bridge](http://wiki.ros.org/cv_bridge)__, __[sensor_msgs](http://wiki.ros.org/sensor_msgs)__, __[std_msgs](http://wiki.ros.org/std_msgs)__, __[rclpy](http://wiki.ros.org/rclpy)__
* __[Gazebo Harmonic](https://gazebosim.org/)__ 8.11.0 with __[ros_gz_bridge](https://github.com/gazebosim/ros_gz)__
* __[PX4-Autopilot](https://github.com/PX4/PX4-Autopilot)__ with __[px4_msgs](https://github.com/PX4/px4_msgs)__
* __[Micro-XRCE-DDS-Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent)__ (ROS 2 ↔ PX4 bridge)
* __[QGroundControl](https://qgroundcontrol.com/)__ (지상 관제 소프트웨어)
* __[NumPy](https://numpy.org/)__ < 2.0 (OpenCV 호환성)
* __[OpenCV-Python](https://opencv.org/)__ with __cv2.aruco__ (Aruco 마커 인식)
* __[OpenAI Python SDK](https://github.com/openai/openai-python)__ (자연어 명령 처리)
* __[Pygame](https://www.pygame.org/)__ (미션 UI)

---

## External Dependencies
이 저장소에는 아래 항목들이 포함되어 있지 않습니다. 별도로 설치해야 합니다.
- PX4-Autopilot
- px4_msgs (ros2 파일 쪽에 설치, Ex. capstone_ws)
- QGroundControl (여기서는 Downloads 폴더에 다운받았음)
- MicroXRCEAgent (별도 설치 필요)
- OpenAI API Key (chat_mission_ui 실행 시 필요)

<br>

아래의 터미널 명령문은 위의 내용을 기반으로 작성되었습니다.

<br>

또한 `px4vision_model.sdf` 파일을 아래 경로에 복사해야 합니다.
~~~bash
cp px4vision_model.sdf ~/PX4-Autopilot/Tools/simulation/gz/models/px4vision/model.sdf
~~~
이 파일에는 드론 하단에 Aruco 마커 인식용 하향 카메라(`down_camera_link`)가 추가되어 있습니다.

<br><br>

---

## Build
코드를 수정한 경우에는 다시 build 해야 합니다.
~~~bash
cd ~/capstone_ws
colcon build --packages-select warehouse_offboard
source install/setup.bash
~~~
처음 새로운 PC에서 workspace를 세팅할 때는 ROS 2 underlay를 먼저 source 하는 것이 좋습니다.
~~~bash
cd ~/capstone_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select warehouse_offboard
source install/setup.bash
~~~

---
<br>

## Before Running
ROS 2 패키지를 실행하는 터미널에서는 항상 아래 명령을 먼저 실행합니다.
~~~bash
cd ~/capstone_ws
source install/setup.bash
~~~

---
<br>

## Run Sequence
아래 순서대로 총 9개의 터미널을 사용합니다.

<br><br>

### Terminal 1: Gazebo
~~~bash
cd ~/capstone_ws/src/warehouse_offboard
source ~/.bashrc
export GZ_SIM_RESOURCE_PATH=~/PX4-Autopilot/Tools/simulation/gz/models:~/PX4-Autopilot/Tools/simulation/gz/worlds
gz sim -r ~/capstone_ws/src/warehouse_offboard/worlds/warehouse.sdf
~~~

<br><br>

### Terminal 2: PX4 (Terminal 1 완전히 뜬 후 실행)
~~~bash
cd ~/PX4-Autopilot
source ~/.bashrc
PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4006 PX4_SIM_MODEL=gz_px4vision PX4_GZ_MODEL_POSE="2.1,-1.5,0.3,0,0,0" ./build/px4_sitl_default/bin/px4
~~~

<br><br>

### Terminal 3: MicroXRCEAgent (ROS ↔ PX4 bridge)
~~~bash
MicroXRCEAgent udp4 -p 8888
~~~

<br><br>

### Terminal 4: QGroundControl
~~~bash
cd ~/Downloads
./QGroundControl.AppImage
~~~

<br><br>

### Terminal 5: goto_point (드론 미션 백엔드)
~~~bash
cd ~/capstone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run warehouse_offboard goto_point --ros-args --params-file ~/capstone_ws/src/warehouse_offboard/params/goto_point.yaml
~~~

<br><br>

### Terminal 6: chat_mission_ui (자연어 명령 UI)
~~~bash
export OPENAI_API_KEY="your_api_key"
cd ~/capstone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run warehouse_offboard chat_mission_ui
~~~

<br><br>

### Terminal 7: gz_camera_bridge (Gazebo ↔ ROS2 카메라 브릿지)
~~~bash
cd ~/capstone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run warehouse_offboard gz_camera_bridge
~~~

<br><br>

### Terminal 8: rqt_image_view (카메라 영상 확인)
~~~bash
source /opt/ros/humble/setup.bash
ros2 run rqt_image_view rqt_image_view
~~~
뜨면 드롭다운에서 `/aruco_land/debug_image` 선택

<br><br>

### Terminal 9: aruco_land (Aruco 마커 기반 정밀 착륙)
~~~bash
cd ~/capstone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run warehouse_offboard aruco_land --ros-args --params-file ~/capstone_ws/src/warehouse_offboard/params/goto_point.yaml
~~~

<br><br>

---

## Waypoints
| 이름 | 랙 위치 (world) | 통로 위치 (world) |
|------|----------------|------------------|
| A-01 | (0.5, -1.7)    | (0.5, -0.8)      |
| A-02 | (-1.7, -1.7)   | (-1.7, -0.8)     |
| A-03 | (0.5, 0.7)     | (0.5, 1.4)       |
| A-04 | (-1.7, 0.7)    | (-1.7, 1.4)      |

- 이륙/착륙 지점: (2.1, -1.5), 고도: 1.8m
- Aruco 마커 (ID=0): 이착륙 패드 중앙 배치

<br>

---

## System Architecture
~~~
[chat_mission_ui] ── /mission_target_name ──▶ [goto_point] ──▶ PX4 (offboard)
      │                                             │
  LLM/규칙 기반                             /mission_status_text
  (llm_selector)                                   │
                                            ◀──────┘
[aruco_land] ◀── /mission_status_text (PRELAND_SETTLE)
      │
  하단 카메라 → Aruco 마커 인식 → 정밀 착륙
      │
  /aruco_land/status (ARUCO_LAND_DONE) ──▶ [goto_point]

[gz_camera_bridge] ──▶ /camera/image_raw
                   ──▶ /camera/down_image_raw ──▶ rqt_image_view
~~~

<br>

---

## Notes
- 코드를 수정한 뒤에는 반드시 다시 build 해야 합니다.
- `goto_point.yaml` 파일에서 waypoint 및 aruco_land 관련 파라미터를 수정할 수 있습니다.
- QGroundControl AppImage 위치가 다르면 경로를 맞게 수정해야 합니다.
- PX4 경로가 `~/PX4-Autopilot`이 아닐 경우 해당 경로로 수정해야 합니다.
- `llm_selector.py`는 `OPENAI_API_KEY` 환경변수를 사용합니다. Terminal 6 실행 전 반드시 설정해야 합니다.
- ROS_DOMAIN_ID가 팀원과 겹치지 않도록 설정하세요.
예시:
~~~bash
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
~~~
