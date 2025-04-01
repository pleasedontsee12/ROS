# 인공지능과 ROS 2를 활용한 안내 로봇(SerBot AGV) 개발 및 운용

## 1. 과제 목표
- 사용자가 지정한 목적지까지 SerBot AGV가 자율적으로 안전하게 이동하여 안내하는 로봇 시스템을 개발하고 운용한다.

- ROS 2 Foxy 프레임워크를 기반으로 로봇의 이동, 센서 처리, 인공지능 기능을 통합한다.

- 카메라를 이용한 기본적인 환경 인식(사람 인식) 기능을 구현하여 상호작용 또는 안전 기능의 기반을 마련한다.

### 세부 목표:

- SerBot AGV 하드웨어(LiDAR, 모터, 카메라)를 ROS 2 환경에서 구동 및 제어한다.

- LiDAR 센서를 활용하여 로봇 운용 환경의 2D 지도를 작성한다 (SLAM).

- 작성된 지도를 기반으로 로봇의 현재 위치를 실시간으로 추정한다 (Localization).

- ROS 2 Navigation2 스택을 설정하고 활용하여 목적지까지의 경로를 계획하고 장애물을 회피하며 자율 주행한다.

- 카메라 영상에서 사람을 인식하는 기본적인 인공지능 모델을 ROS 2 노드로 통합한다.

- 사용자와의 기본적인 상호작용 인터페이스(목적지 입력)를 구현한다.

- 음성 인식/합성, 디스플레이 출력 등 추가적인 상호작용 기능을 구현한다.

## 2. 개발 내용
### ROS 2 기본 환경 구축 :

- ROS 2 Foxy 설치 확인 및 환경 설정.

- ROS 2 작업 공간(ros2_ws) 생성 및 빌드.

- SerBot AGV 설정을 위한 커스텀 패키지(serbot_agv_bringup) 생성.

### 로봇 하드웨어 인터페이스 :

#### 모터 제어 노드: 
geometry_msgs/msg/Twist 타입의 /cmd_vel 토픽을 구독하여 SerBot AGV의 pop 라이브러리 또는 하위 레벨 제어 명령으로 변환하여 실제 모터를 구동하는 Python 노드 (rclpy 사용).

#### LiDAR 드라이버 연동: 
설치된 rplidar-ros (ROS 2 버전) 패키지를 사용하여 /scan 토픽으로 LiDAR 데이터를 발행하도록 설정.

#### 카메라 드라이버 연동: 
USB 카메라 등을 위한 usb_cam 또는 관련 ROS 2 패키지를 사용하여 /image_raw 토픽으로 카메라 영상 발행 설정.

#### 로봇 모델링 (URDF):

- SerBot AGV의 링크(link), 조인트(joint), 센서 위치 등을 정의하는 URDF(seb.xacro) 파일 작성 및 관리.

- robot_state_publisher 노드를 사용하여 URDF와 /joint_states 토픽을 기반으로 로봇의 TF(Transform) 트리 발행 설정. (실제 관절 센서가 없다면 joint_state_publisher 사용)

#### SLAM (지도 작성):

- slam_toolbox 패키지 설정 및 실행.

- LiDAR 데이터(/scan)와 TF 정보를 사용하여 로봇 운용 환경의 2D 점유 격자 지도(occupancy grid map) 생성.

- 생성된 지도를 파일(map.pgm, map.yaml)로 저장.

### Localization (위치 추정):

- Navigation2의 nav2_amcl 패키지 설정 및 실행.

- 저장된 지도와 실시간 LiDAR 데이터(/scan), TF 정보를 사용하여 지도 상에서 로봇의 실시간 위치와 방향 추정.

### Navigation (자율 주행):

#### Navigation2 스택 (nav2_bringup) 설정:

- Costmap 설정: Global costmap, Local costmap 파라미터 설정 (YAML). 로봇 크기, 센서 정보, 장애물 레이어(LiDAR) 등 정의.

#### Planner/Controller 설정: 
- 경로 계획기(Planner -  NavFn, SmacPlanner) 및 경로 추종 제어기(Controller - DWB, TEB) 파라미터 설정 (YAML). 로봇의 동역학 특성, 속도/가속도 제한 등 반영.

#### Behavior Tree 설정: 
- 네비게이션 작업 흐름(경로 계획->경로 추종->복구 행동 등)을 정의하는 Behavior Tree XML 파일 설정.

- Navigation2 실행 및 목표 지점(Goal) 전달 인터페이스 구현 (RViz2).

#### 인공지능 기능 (사람 인식):

- 카메라 기반 사람 인식 노드: 카메라 영상(/image_raw)을 구독하여 사전 훈련된 딥러닝 모델(YOLO, SSD 등 - OpenCV DNN 모듈 활용)을 통해 영상 내 사람의 위치(Bounding Box)를 감지하고, 그 결과를 커스텀 메시지 또는 vision_msgs 타입의 토픽으로 발행하는 Python 노드.

#### 안내 로봇 통합 로직 및 상호작용:

- 중앙 제어 노드 ("Brain"): 사용자로부터 목적지 입력을 받고, 현재 로봇 상태(위치, 인식 정보 등)를 모니터링하며, Navigation2에 목표 지점을 전달하고, 안내 시나리오(이동, 정지, 안내 메시지 등)를 관리하는 메인 로직 노드.

#### 목적지 입력 인터페이스:

-  ROS 2 토픽으로 목적지 좌표(x, y, yaw)를 발행하는 별도의 노드.


#### 사용자 피드백:

- 음성 합성(TTS) 노드를 통해 안내 메시지 출력.

- 로봇의 디스플레이에 상태 또는 안내 정보 출력.

## 3. 개발 과정
### Phase 1: ROS 2 기반 구축 및 기본 구동 (1-2주차)

- 환경 설정 및 패키지 생성: ROS 2 작업 공간, 커스텀 패키지(serbot_agv_bringup) 생성 및 기본 설정 완료.

- URDF 작성/검증: seb.xacro 파일 작성 및 check_urdf 등으로 검증. robot_state_publisher와 joint_state_publisher (GUI)를 실행하여 RViz2에서 로봇 모델 및 TF 트리 확인.

- 모터 제어 노드 개발: /cmd_vel 수신하여 로봇을 직접 움직이는 Python 노드 개발 및 테스트 (전/후진, 회전).

- LiDAR 연동: rplidar-ros 실행 및 RViz2에서 /scan 데이터 시각화 확인. TF 설정 (base_link <-> laser) 확인.

- 카메라/초음파 연동: 관련 드라이버 실행 및 RViz2에서 데이터 시각화 확인. TF 설정 확인.

### Phase 2: 지도 작성 및 위치 추정 (3주차차)

- SLAM 실행: slam_toolbox 설정 및 실행. 로봇을 수동으로 조작하여 운용 공간 지도 작성. 지도 품질 확인 및 저장.

- Localization 실행: 저장된 지도를 로딩하고 nav2_amcl 설정 및 실행. RViz2에서 로봇의 초기 위치를 설정해주고, 로봇 이동 시 위치 추정이 잘 되는지 확인.

### Phase 3: 자율 주행 설정 및 테스트 (4-5주차)

- Navigation2 설정: 로봇 파라미터에 맞게 Costmap, Planner, Controller, Behavior Tree 설정 파일(YAML, XML) 작성 및 수정. (nav2_bringup 예제 참고)

- Navigation2 실행: 작성된 설정 파일들을 사용하여 Navigation2 스택 전체를 런치 파일로 실행.

- 기본 자율 주행 테스트: RViz2의 "Nav2 Goal" 기능을 사용하여 목표 지점을 설정하고, 로봇이 장애물을 회피하며 목적지까지 자율 주행하는지 테스트 및 파라미터 튜닝.

### Phase 4: 인공지능 및 상호작용 구현 (6-8주차)

- 사람 인식 노드 개발: 카메라 영상을 이용한 사람 인식 Python 노드 개발. 인식 결과(Bounding Box 등)를 토픽으로 발행하고 RViz2 등에서 확인. (OpenCV DNN, YOLO 등 모델 활용)

- 목적지 입력/피드백 인터페이스 개발: 토픽 기반의 간단한 목적지 입력 노드 또는 음성/디스플레이 관련 노드 개발.

- 중앙 제어 노드 ("Brain") 개발: 안내 시나리오 로직 구현. 사용자 입력 처리, Navigation2 목표 전달

### Phase 5: 통합 및 최종 테스트 (9주차)

- 전체 시스템 통합: 개발된 모든 노드(하드웨어 제어, SLAM/Localization, Navigation, AI, 상호작용)를 하나의 메인 런치 파일로 통합.

- 안내 시나리오 테스트: 실제 환경에서 사용자가 목적지를 지정하고 로봇이 안내하는 전체 과정을 테스트. 다양한 예외 상황(장애물, 사람 등장 등)에 대한 로봇의 반응 확인 및 개선.

- 성능 평가 및 문서화: 로봇의 주행 안정성, 안내 성공률 등 평가. 개발 과정 및 결과 문서화.

4. 역할 분담 
### 팀원 1: 센서 인터페이스 & AI 인식
#### 주요 역할
-  LiDAR, 카메라 등 센서의 ROS 2 드라이버 설정 및 데이터 발행 노드 개발/관리. 카메라 기반 사람 인식 AI 노드 개발 및 구현.

#### 세부 내용:
- rplidar-ros (ROS 2) 설정 및 /scan 토픽 발행 확인.
카메라 드라이버 (usb_cam 등) 설정 및 /image_raw 토픽 발행 확인.
- 카메라 영상 구독, 사람 인식 모델(YOLO 등) 적용, 인식 결과(Bounding Box 등)를 커스텀 토픽으로 발행하는 노드 개발.
관련 센서의 URDF 링크 정의 및 TF 발행 설정 지원 (팀원 3과 협력).

#### 필요 기술: 
- ROS 2 기본, Python(rclpy), 센서 데이터 처리, 컴퓨터 비전 기초, 딥러닝 모델 활용(OpenCV DNN 등).

### 팀원 2: 액추에이터 & 중앙 제어
#### 주요 역할: 
- 모터 제어 노드 개발. 전체 안내 시나리오를 관리하는 중앙 제어("Brain") 노드 개발.

#### 세부 내용:
- /cmd_vel 구독, pop 라이브러리 등 활용하여 실제 모터 제어하는 Python 노드 개발 및 안정화.
- 로봇 상태(위치, AI 인식 결과 등) 구독, 목적지 관리, Navigation2 목표 전달, 안내 상태(이동 중, 도착, 대기 등) 관리 로직 구현.
- 간단한 목적지 입력 메커니즘 구현
액추에이터(바퀴) 관련 URDF 링크/조인트 정의 지원 (팀원 3 협력).

#### 필요 기술: 
- ROS 2 기본, Python(rclpy), 로봇 제어 로직, 상태 관리, 간단한 알고리즘.

### 팀원 3: 로봇 모델링 & TF 관리
#### 주요 역할: 
- SerBot AGV의 정확한 URDF (seb.xacro) 모델 작성 및 유지보수. 전체 시스템의 TF(Transform) 트리 관리 및 디버깅.

#### 세부 내용:
- 로봇의 모든 링크(몸체, 바퀴, 센서 마운트 등)와 조인트(fixed, continuous 등) 상세 정의.
- Visual, Collision, Inertial 정보 포함 (Inertial은 추정치 사용 가능).
- robot_state_publisher 설정 및 실행 관리.
- joint_state_publisher 설정 (실제 관절 센서 없을 시).
RViz2에서 로봇 모델과 TF 트리 시각화 및 정확성 검증. 다른 팀(센서, 액추에이터)과 협력하여 TF 설정 일관성 유지.
#### 필요 기술: 
- ROS 2 기본, URDF/Xacro 문법, TF 개념 이해, 3D 모델링 개념, RViz2 활용.

### 팀원 4: SLAM (지도 작성)
#### 주요 역할: 
- SLAM 알고리즘(예: slam_toolbox)을 사용하여 로봇 운용 환경의 2D 지도 생성 및 최적화.

#### 세부 내용:
- slam_toolbox 패키지 설정 (파라미터 튜닝: LiDAR 매칭, 루프 클로징 등).
- LiDAR(/scan) 및 TF 데이터를 입력받아 SLAM 실행.
- 로봇 수동 조작 지원 또는 자체 탐색 로직 활용하여 지도 작성.
- 생성된 지도의 품질 평가 및 개선 (파라미터 조정, 재탐색 등).
최종 지도를 map_server 형식(.pgm, .yaml)으로 저장 및 관리.

#### 필요 기술: 
- ROS 2 기본, SLAM 이론 이해, slam_toolbox 사용법, 지도 데이터 형식 이해.

### 팀원 5: Localization & TF 검증
#### 주요 역할: 
- 생성된 지도를 기반으로 로봇의 실시간 위치 추정 (nav2_amcl) 설정 및 성능 최적화. TF 트리 전반적인 모니터링 및 문제 해결 지원.

#### 세부 내용:
- nav2_amcl 패키지 설정 (파라미터 튜닝: 파티클 수, 센서 모델, 오도메트리 모델 등).
- 저장된 지도 로딩 (map_server 활용).
- LiDAR(/scan) 및 TF 데이터를 입력받아 위치 추정 실행.
RViz2에서 로봇 초기 위치 설정 및 위치 추정 정확도/안정성 평가.
- AMCL이 발행하는 map -> odom TF 확인. 전체 TF 트리 (rqt_tf_tree 등 활용) 이상 유무 지속적 모니터링.
#### 필요 기술: 
- ROS 2 기본, Localization 이론(AMCL) 이해, nav2_amcl 사용법, TF 개념 심화 이해, 디버깅 능력.

### 팀 6: Navigation2 스택 설정
#### 주요 역할: 
- ROS 2 Navigation2 스택의 핵심 컴포넌트(Costmap, Planner, Controller, Behavior Tree) 설정 및 튜닝.

#### 세부 내용:
- Global/Local Costmap 설정 (YAML): 로봇 크기, 센서 레이어(LiDAR, (옵션) 초음파 RangeSensorLayer) 구성, 팽창 반경 등 정의.
- 경로 계획기(Global Planner - 예: SmacPlanner) 및 경로 추종 제어기(Local Planner/Controller - 예: DWB) 파라미터 설정 (YAML). 속도/가속도 제한, 경로 추종 정확도 등 튜닝.
- Behavior Tree (XML): 네비게이션 작업 흐름 정의 및 필요시 커스터마이징. 복구 행동(Recovery behaviors) 설정.
Navigation2 전체 스택 런치 파일 (nav2_bringup 기반) 작성 및 관리.
- 자율 주행 테스트 결과 기반으로 파라미터 지속적 튜닝.

#### 필요 기술: 
- ROS 2 기본, Navigation2 아키텍처 이해, Costmap/Planner/Controller 개념 이해, YAML/XML 설정, Behavior Tree 기본, 성능 튜닝 경험.


