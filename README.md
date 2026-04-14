# warehouse_offboard

ROS 2 Humble, Gazebo, PX4 SITL 환경에서 창고 월드 기반 드론 오프보드 미션을 실행하기 위한 패키지입니다.

## Environment

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo
- PX4 SITL
- QGroundControl
- MicroXRCEAgent

## Repository Structure

```bash
warehouse_offboard/
├── package.xml
├── params/
│   └── goto_point.yaml
├── resource/
│   └── warehouse_offboard
├── setup.cfg
├── setup.py
├── test/
├── warehouse_offboard/
│   ├── __init__.py
│   ├── chat_mission_ui.py
│   ├── goto_point.py
│   └── llm_selector.py
└── worlds/
    └── warehouse.sdf
