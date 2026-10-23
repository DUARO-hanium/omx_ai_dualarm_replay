# OMX-AI dual-arm replay

OMX-AI 팔로워 두 대의 관절·그리퍼 궤적을 CLI로 동시에 재생합니다.
**`data/package_data`는 기존 take04의 원본 데이터**이며, 다른 녹화 데이터는 포함하지 않습니다.
2,764프레임·92.10초, 좌우 CSV와 장치 좌표 정보로 구성됩니다.

## Clone 후 실행

Ubuntu 24.04 x86_64 PC에 녹화에 사용한 같은 팔로워 두 대를 연결합니다.
USB 고유 ID로 좌우를 자동 인식합니다. 리더·카메라·Cyclo UI는 사용하지 않습니다.

```bash
git clone https://github.com/DUARO-hanium/omx_ai_dualarm_replay.git
cd omx_ai_dualarm_replay
./replay.sh package_data
```

첫 실행은 필요한 Docker를 공식 Ubuntu 저장소에서 설치하고(sudo 비밀번호 입력),
고정 digest의 ROBOTIS 이미지로 ROS 2 Jazzy 환경을 자동 빌드합니다. 인터넷 연결이 필요합니다.
호스트 ROS 설치나 수동 패치·설정 파일 수정은 필요하지 않습니다.
기존에 실행한 로봇 제어 프로그램은 먼저 종료합니다.

`play` + Enter를 입력하면 다음 순서로 진행합니다.

1. 현재 자세에서 토크를 활성화하고 첫 기록 자세까지 기본 10초 동안 접근합니다.
2. 좌우 CSV를 공통 시작 시각에 원래 속도로 재생합니다.
3. **마지막 기록 자세를 토크 ON으로 유지하고 재생 CLI는 종료됩니다.**

자세 유지를 위한 ROS 컨트롤러는 백그라운드 컨테이너에서 계속 실행됩니다.
첫 자세로 자동 복귀하지 않습니다. 토크 해제는 별도 CLI로 실행합니다.

```bash
# 양팔을 손으로 받친 다음 실행: 그리퍼 포함 12개 모터 OFF 확인 및 제어 종료
./torque_off.sh
```

토크 해제 후에 다음 재생을 시작할 수 있습니다. 재생 중 Ctrl+C는 현재 자세 유지 명령을
보내며, 토크 해제는 `torque_off.sh`에서 처리합니다. USB·전원 장애에서의 비상 정지를 보장하지 않습니다.

```bash
./replay.sh --status                         # 재생/자세 유지 상태
./replay.sh package_data --speed 0.5          # 절반 속도
./replay.sh package_data --inspect            # CSV 확인, USB 접근 없음
./replay.sh package_data --check              # 장치·좌표·오류·토크 OFF 확인
./replay.sh package_data --mock --speed 20 --align 1  # USB 없는 모의 재생
./torque_off.sh                              # 모의 세션도 같은 CLI로 종료
```

## 재생 방식과 데이터

[replay_final](https://github.com/DUARO-hanium/replay_final)의 ROS 2 재생 방식을 사용합니다.
CSV 각도를 `JointTrajectory`로 만들어 `JointTrajectoryController`에 전달하며,
원본 단일 팔 재생기의 현재 자세 → 첫 기록 자세 접근 구간을 유지합니다.
양팔에는 서로 다른 `left.csv`·`right.csv`와 동일한 궤적 시작 timestamp를 적용합니다.

CSV는 `time,joint1,joint2,joint3,joint4,joint5,gripper_joint_1`, 각도 단위는 radian입니다.
각도·시간·고정 구간을 수정하거나 클리핑하지 않습니다. 기본 `--speed 1`은 기록 시간을 유지합니다.
모터 모델·homing offset·회전 방향이 기록 당시와 다르면 실행을 막습니다.
다른 보드는 `--left-port 경로 --right-port 경로`로 지정할 수 있으나 기구와 모터 좌표가 같아야 합니다.

팔 관절은 확장 위치 모드(4), 그리퍼는 전류 기반 위치 모드(5), 그리퍼 전류는 100 mA입니다.
기존 단일 회전 위치 제한과 그리퍼 보정 범위는 적용하지 않으며 모터 오류 보호는 유지합니다.
실제 추종 오차·파지력은 기구와 작업 환경에 영향을 받습니다. 첫 자세 접근은 장애물을 고려하지
않으므로 이동 경로를 비워 둡니다. `results/`의 실행 상태·로그는 Git에서 제외됩니다.
출처와 변경 사항은 `NOTICE`에 있습니다.
