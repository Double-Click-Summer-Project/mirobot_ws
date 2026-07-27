# Mirobot ROS 2 Driver

ROS 2 Humble에서 WLKATA Mirobot 한 대의 Homing, 6축 조인트, XYZ 좌표 및 진공 펌프를 제어하기 위한 최소 드라이버 패키지이다.

비전, Master/Slave, 컨베이어, RViz 및 프로젝트 전용 로직은 포함하지 않는다.

---

# 1. Git 클론 및 워크스페이스 생성

## 1-1. ROS 2 워크스페이스 생성

```bash
mkdir -p ~/mirobot_ws/src
cd ~/mirobot_ws
```

---

## 1-2. GitHub에서 패키지 클론

```bash
cd ~/mirobot_ws/src

git clone https://github.com/your-org/mirobot_driver.git
```

> 실제 사용 시 `your-org`는 본인 GitHub 주소로 변경

---

## 1-3. 브랜치 확인 (선택)

```bash
cd mirobot_driver

git branch
git checkout main
```

---

# 2. 패키지 구조

```text
mirobot_ws/
├── src/
│   └── mirobot_driver/
│       ├── config/
│       │   └── mirobot.yaml
│       ├── launch/
│       │   └── mirobot.launch.py
│       ├── mirobot_driver/
│       │   ├── driver.py
│       │   ├── protocol.py
│       │   ├── cli_common.py
│       │   ├── joint_command.py
│       │   └── xyz_command.py
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
├── build/
├── install/
└── log/
```

* `driver.py`: 시리얼 연결, Homing, 제어 명령 및 상태 관리
* `protocol.py`: ROS 데이터를 Mirobot G-code로 변환
* `joint_command.py`: 6축 조인트 명령 입력
* `xyz_command.py`: XYZ 좌표 명령 입력
* `mirobot.yaml`: 조인트 및 XYZ 범위, 속도, timeout 설정
* `mirobot.launch.py`: 드라이버 실행 및 옵션 관리

---

# 3. 의존성 설치

## 3-1. ROS 2 환경 설정

```bash
source /opt/ros/humble/setup.bash
```

---

## 3-2. Python 의존성 설치

```bash
cd ~/mirobot_ws/src/mirobot_driver

pip3 install -r requirements.txt
```

(없을 경우 기본 설치)

```bash
pip3 install pyserial
```

---

# 4. 빌드

```bash
cd ~/mirobot_ws

source /opt/ros/humble/setup.bash

colcon build --packages-select mirobot_driver

source install/setup.bash
```

---

## 패키지 확인

```bash
ros2 pkg prefix mirobot_driver
```

---

# 5. 드라이버 실행

```bash
ros2 launch mirobot_driver mirobot.launch.py \
  port:=/dev/ttyUSB0 \
  auto_home:=true \
  enable_motion_after_auto_home:=true
```

---

## 실행 순서

```text
USB 시리얼 연결
→ Homing 수행
→ Idle 상태 확인
→ 조인트 및 XYZ 명령 활성화
```

---

# 6. 조인트 제어

```bash
ros2 run mirobot_driver joint_command --degrees \
  0 -20 30 0 0 0
```

---

# 7. XYZ 제어

```bash
ros2 run mirobot_driver xyz_command --millimeters \
  200 0 150
```

---

# 8. 펌프 제어

## 펌프 ON

```bash
ros2 service call /mirobot/set_pump \
  std_srvs/srv/SetBool "{data: true}"
```

## 펌프 OFF

```bash
ros2 service call /mirobot/set_pump \
  std_srvs/srv/SetBool "{data: false}"
```

---

# 9. 상태 확인

```bash
ros2 topic echo --once /mirobot/is_busy
ros2 topic echo --once /mirobot/connected
ros2 topic echo --once /mirobot/motion_enabled
```

---

# 10. 동작 규칙

* Homing 완료 전에는 모든 모션 명령이 차단됨
* `/mirobot/motion_enabled == true` 이후 제어 가능
* `/mirobot/is_busy == false`일 때 다음 명령 전송 권장
* 안전을 위해 펌프 OFF는 종료 시 반드시 수행

---

# 11. 전체 실행 흐름 요약

```text
git clone
→ workspace 생성
→ dependency 설치
→ colcon build
→ launch 실행
→ homing
→ joint / xyz control
→ pump control
→ status monitoring
```
