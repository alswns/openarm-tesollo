# OpenArm V2 + DG-5F-S: RViz MoveIt 연습

OpenArm V2 양팔과 DG-5F-S 20자유도 양손을 RViz에 표시하고, **MoveIt 기본 Interactive Marker로 목표를 지정한 뒤 Plan/Execute 버튼으로 움직이는** 연습용 구성입니다. 마커는 RViz의 MotionPlanning 플러그인이 생성하며, 경로계획 모드에는 별도 마커 처리 노드가 없습니다. Gazebo는 실행하지 않으며 `ros2_control` mock 하드웨어를 사용합니다. 실제 로봇 제어용 설정이 아닙니다.

## 설치 및 실행

macOS ARM 또는 Ubuntu 24.04에서 RoboStack ROS 2 Jazzy 환경을 사용할 수 있습니다. 아래 명령은 이 폴더에서 실행합니다.

```bash
micromamba create -y -p ./.conda -c conda-forge -c robostack-jazzy \
  --strict-channel-priority ros-jazzy-ros-base ros-jazzy-rviz2 \
  ros-jazzy-moveit-servo ros-jazzy-moveit-kinematics \
  ros-jazzy-moveit-ros-move-group ros-jazzy-moveit-ros-visualization \
  ros-jazzy-moveit-planners-ompl ros-jazzy-moveit-simple-controller-manager \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-joint-state-publisher-gui ros-jazzy-robot-state-publisher \
  ros-jazzy-interactive-markers ros-jazzy-xacro ros-jazzy-ros-gz \
  colcon-common-extensions vcstool
./scripts/fix_robostack_macos.sh
source .conda/setup.sh
vcs import src < models.repos
npm ci --prefix src/openarm_tesollo_moveit/web --ignore-scripts
npm run build --prefix src/openarm_tesollo_moveit/web
colcon build --symlink-install --packages-up-to openarm_tesollo_moveit \
  --cmake-args -DPython3_EXECUTABLE="$PWD/.conda/bin/python"
source install/setup.sh
ros2 launch openarm_tesollo_moveit practice.launch.py
```

공식 OpenArm 프리셋의 **ZED Mini와 카메라 마운트**를 포함합니다. 웹 3D 렌더러 빌드에는 Node.js와 npm이 필요합니다. Three.js와 URDFLoader는 로컬 번들로 제공하며 CDN은 사용하지 않습니다.

`models.repos`는 공식 `openarm_ros2`도 고정 커밋으로 가져옵니다. MoveIt 설정은 `openarm_bimanual_moveit_config`의 V2 SRDF·IK·관절 한계·컨트롤러 설정과 공식 launch처럼 `MoveItConfigsBuilder`의 OMPL 기본값을 사용합니다. 공식 demo launch 자체는 기본 집게와 다른 로봇 설명을 전제로 하므로 그대로 실행하지 않습니다. 대신 통합 모델에 맞춰 집게 항목을 DG 손가락 그룹으로 대체하고, 팔 체인 끝을 손등 TCP로 바꾸며, mock 컨트롤러 이름을 연결합니다. IK 시간 제한은 기존과 같은 0.05초, 관절 속도는 URDF 한계 이하, 가속도는 기존 1.0 이하로 제한합니다. 공식 이름이 바뀌면 재검토가 필요합니다.

RViz에서 **Interact** 도구를 선택하고 MoveIt 목표 마커의 화살표나 원을 드래그합니다. 목표를 지정한 뒤 MotionPlanning 패널에서 **Plan → Execute**를 누릅니다. 마우스를 놓는 것만으로 자동 실행되지 않습니다. 기본 Planning Group은 `bimanual`이라 양팔 목표 마커가 표시됩니다. `left_arm`, `right_arm`으로 바꾸면 해당 팔만 표시됩니다. 드래그 중에는 실제 로봇이 움직이지 않습니다.

모든 관절은 `0 rad`로 시작합니다. 경로계획 모드는 Servo를 실행하지 않아 Servo 특이점 정지에 막히지 않지만, 도달 불가능한 목표나 충돌하는 경로는 실패할 수 있습니다. 손가락 관절은 목표 마커에 의해 자동으로 구부러지지 않습니다.

RViz의 **Allow Approximate IK**는 목표 로봇 미리보기에 사용합니다. 목표 상태 관리와 계획 요청은 MoveIt 기본 플러그인이 처리합니다. 실제 경로계획에서는 충돌 검사가 유지됩니다.

현재 카메라를 포함한 전체 모델의 초기 상태 유효성 검사에서는 카메라 마운트와 몸체·팔 베이스 등의 충돌이 보고됩니다. 팔 그룹 계획이 성공하더라도 전체 모델의 무충돌을 보장하지 않습니다. 접촉 형상과 충돌 행렬은 실제 장착 구조를 기준으로 검증해야 하며, 임의로 충돌을 무시하지 않았습니다.

모델을 수정한 뒤에는 **기존 launch를 Ctrl+C로 종료하고** 다시 실행하세요. 같은 ROS 도메인에서 두 번 실행하면 목표 명령·관절 상태가 충돌할 수 있습니다.

## 웹 6D pose 텔레옵 (RViz mock 전용)

기존 launch를 종료한 뒤 다음 명령으로 실행하고 `http://127.0.0.1:8765`를 엽니다.

```bash
ros2 launch openarm_tesollo_moveit practice.launch.py control_mode:=web
```

웹에서 좌우 팔을 선택하고 **실시간 추종**을 켭니다. 파란 점을 XY/XZ/YZ 평면에서 드래그하거나 XYZ 입력과 Roll/Pitch/Yaw를 조절하면 `world` 기준 손등 중앙 TCP의 목표 pose가 Servo에 전달됩니다. 노란 점은 실제 TCP 위치입니다. 위치는 m, 회전 입력은 degree이며 ROS에는 quaternion으로 전달합니다. 초기화 시점의 위치 기준 XYZ 각 축 **±1 m**, 자세 기준 Roll/Pitch/Yaw 각 축 **±180°**의 입력 영역으로 시작합니다. 입력 영역은 실제 도달 가능한 작업 공간을 뜻하지 않습니다.

제어점은 `left_hand_back_tcp`, `right_hand_back_tcp`입니다. 기존 손목 쪽 `*_hand_link_base`에서 손 베이스 로컬 좌표 `xyz="-0.01804 0 0.05"`로 이동한 손등 중앙 표면에 고정하며, 축 방향은 손 베이스와 같습니다. 공식 손 메시의 손등 표면을 기준으로 잡은 연습용 TCP이지 실측 보정값은 아닙니다. 손 장착 오프셋은 바꾸지 않습니다. RViz와 웹 로봇 모델의 작은 파란 점(왼손)·주황 점(오른손)이 이 TCP를 표시합니다. MoveIt 경로계획과 선택적인 Servo 마커도 동일한 TCP를 사용합니다.

확장한 입력을 받도록 서버의 기존 20 cm 거리 제한은 3.5 m로 넓히고, 1.2 rad 자세 차이 제한은 제거했습니다. 유한 숫자·단위 quaternion 검증과 `world` 원점 기준 각 축 ±3 m의 입력 검증은 유지합니다. 도달 불가능한 목표를 입력해도 관절 한계·특이점 보호를 우회하지 않습니다.

웹 모드만 `config/web_initial.yaml`의 굽힌 초기 자세를 사용합니다. 기본 경로계획·Servo 모드의 초기 각도는 여전히 모두 0입니다. 이 모드에는 RViz 마커 서버가 없으며 웹 입력만 사용합니다.

브라우저는 목표를 20 Hz로 보내고 ROS 노드는 최신 입력을 50 Hz로 발행합니다. 입력이 0.25초 동안 끊기면 목표 발행을 중단하고 Servo의 명령 timeout으로 정지합니다. 정지 버튼, 탭 숨김, 팔 전환 시에도 목표 발행을 중단합니다. 정지 버튼은 하드웨어 비상정지가 아닙니다. 서버는 localhost에만 바인딩하며, 실시간 제어의 지연 상한은 보장하지 않습니다.

웹 모드의 Servo 선속도 상한은 `1.2 m/s`, 회전 속도 상한은 `1.2 rad/s`입니다. 관절 속도 제한·특이점 감속·필터링 등에 따라 실제 손 속도는 이보다 낮을 수 있습니다. 특이점·관절 한계 보호는 유지하지만, 다른 mock Servo 모드와 마찬가지로 **충돌 검사가 꺼져 있습니다. 실제 로봇에 사용하지 마세요.**

웹 아래의 **가상 카메라**는 `openarm_body_zed_left_camera_frame_optical`에서 보는 3D 화면입니다. 로봇의 실제 ROS 관절 상태로 공식 DAE/STL 모델을 움직이며, **실제 카메라 영상이나 센서 시뮬레이션은 아닙니다**. Gazebo와 카메라 드라이버를 실행하지 않습니다. 표시용 시야각을 사용하므로 실제 카메라의 보정된 영상과 일치하지 않습니다. `전체 로봇` 시점으로 전환하면 장착된 카메라까지 볼 수 있습니다.

## Servo 모드 (선택)

기존 별도 마커와 Servo 연결은 다음 명령으로 실행합니다. 경로계획 모드와 동시에 실행하지 마세요.

```bash
ros2 launch openarm_tesollo_moveit practice.launch.py control_mode:=servo
```

이 모드만 별도 마커 서버를 사용합니다. 마커를 놓아도 마지막 목표를 계속 추종합니다. 상태는 `/left_servo/status`, `/right_servo/status`에서 확인합니다.

현재 Servo 속도 상한은 선속도 `0.4 m/s`, 회전 속도 `0.8 rad/s`입니다. 특이점에 접근하거나 관절 한계에 가까워지면 Servo가 자동으로 감속하거나 정지합니다.

초기 관절 각도는 양팔·양손 모두 `0 rad`입니다. 이 자세에서는 양팔이 특이점에 가까워 자세 추종이 정지할 수 있습니다. 손목 마커의 구체를 우클릭하고 **Joint jog (escape singularity) → Joint 번호 → + 또는 -**를 선택해 개별 관절을 움직일 수 있습니다. 한 번 선택하면 `0.2 rad/s`로 약 `0.5초` 명령하며, 관절 제한과 필터링 때문에 실제 이동량은 표시된 `0.1 rad`와 다를 수 있습니다. 이동 후 마커가 실제 손 위치로 재설정되고 Servo가 자세 모드로 돌아갑니다. 조그 중에는 마커 드래그를 무시합니다. 조그로 움직일 수 있어도 자세 추종이 가능한 상태가 된다는 보장은 없습니다. 관절 한계 보호는 유지됩니다.

현재 조립 모델은 확인되지 않은 링크 간 충돌로 Servo가 시작 직후 정지하므로, mock RViz 연습을 위해 충돌 검사를 끄고 있습니다. 이 설정은 실제 로봇에 사용하지 마세요. 하드웨어에 연결하기 전에 장착 오프셋과 충돌 행렬을 검증하고 `check_collisions`를 다시 켜야 합니다.

마커는 손등 중앙 TCP의 자세를 조작합니다. 손가락 20개 관절은 각 손의 `joint_trajectory_controller`로 따로 제어하며, 마커를 드래그해도 자동으로 구부러지지 않습니다.

## 모델 구성 및 한계

- 통합 Xacro가 공식 `openarm_description`의 조립 매크로를 불러옵니다. 원본 집게는 제외하고, 몸체·양팔·ZED Mini·카메라 마운트를 포함합니다.
- 공식 DG-5F-S 배포판은 좌우에 같은 이름을 쓰는 URDF를 제공합니다. 빌드 시 각 URDF에서 이름에 접두사를 붙이는 Xacro 매크로를 생성하고, 통합 Xacro가 두 매크로를 불러옵니다. 원본 모델을 합친 파일을 저장하지 않습니다.
- 손목의 `xyz="0 0 -0.03"`, `rpy="3.14 0 0"`은 현재 지정된 **임시 부착값**입니다. 실제 어댑터 형상과 방향은 확인되지 않았으므로 하드웨어에는 사용하지 마세요.
- SRDF는 통합 모델에 존재하는 공식 충돌 제외 항목과 실제 연결된 인접 링크의 제외 항목을 사용합니다. 경로계획 모드는 충돌 검사를 사용하며, 선택적인 Servo 모드는 충돌 검사가 꺼져 있습니다.

## 이전 구현 보존

공식 설정 통합 전 코드는 Git의 `previous-implementation` 브랜치에 보존되어 있습니다. 모델 저장소와 빌드 산출물은 커밋에 포함하지 않으며, 모델은 해당 브랜치의 `models.repos`로 복원할 수 있습니다. 이전 코드로 전환한 뒤에는 해당 브랜치에 맞게 다시 빌드해야 합니다.
