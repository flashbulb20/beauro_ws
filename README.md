# 💄 Beauro (Beauty + Robotics)
> **Doosan M0609 협동로봇을 이용한 화장품 원료 배합 자동화 시스템**

![Robot Status](https://img.shields.io/badge/Robot-Doosan_M0609-blue?style=for-the-badge&logo=robot)
![ROS2](https://img.shields.io/badge/ROS2-Foxy%2FHumble-green?style=for-the-badge&logo=ros)
![React](https://img.shields.io/badge/Frontend-React-61DAFB?style=for-the-badge&logo=react)
![Firebase](https://img.shields.io/badge/DB-Firebase_Realtime-FFCA28?style=for-the-badge&logo=firebase)

**Beauro**는 화장품 R&D 및 샘플 제조 과정의 반복적인 수작업을 자동화하여 연구원의 피로도를 줄이고, 실험 데이터(DoE)의 정밀도와 재현성을 보장하기 위해 개발된 로봇 시스템입니다.

---

## 📖 목차
1. [프로젝트 개요](#-프로젝트-개요)
2. [시스템 아키텍처](#-시스템-아키텍처)
3. [주요 기능](#-주요-기능)
4. [하드웨어 구성](#-하드웨어-구성)
5. [설치 및 실행 가이드](#-설치-및-실행-가이드)
6. [시연 영상/스크린샷](#-시연-영상스크린샷)

---

## 🎯 프로젝트 개요

### 🛑 문제 정의 (Problem)
- **반복 노동:** 다수의 샘플(6-Well) 배합 시 연구원의 피로도 증가.
- **휴먼 에러:** 미세한 용량 차이로 인한 실험 데이터 신뢰성 저하.
- **안전 문제:** 미세 파우더 비산 및 화학 액체 취급 시 위험 노출.

### 💡 해결 방안 (Solution)
- **무인 자동화:** 원료 계량, 분주, 교반, 배출 전 과정 자동화.
- **가변 레시피:** Web UI를 통해 각 Well 별 상이한 배합 비율 설정 가능.
- **정밀 제어:** Doosan M0609 로봇을 이용한 ±2% 이내의 정밀 배합.

---

## 🏗 시스템 아키텍처

본 시스템은 **Frontend(Web UI)**, **Backend(Monitor)**, **Robot Node(Executor)**가 **Firebase Realtime Database**를 통해 실시간으로 상태를 동기화하는 구조입니다.

```mermaid
```

| 컴포넌트 | 역할 | 기술 스택 |
|----------|------|-----------|
| Frontend | 레시피 입력, 로봇 제어(Start/Pause/Stop), 실시간 모니터링 | React, TypeScript, TailwindCSS |
| Backend | 로봇 상태(Joint/TCP) 0.1초 단위 감시, 에러 감지 및 물리적 복구 | Python, ROS2, rclpy |
| Executor | 실제 공정 로직(Liquid → Powder → Mix) 수행, 세부 동작 제어 | Python, DSR Library |
| Firebase | 시스템 간 초저지연 상태 동기화 및 명령 전달 | Firebase Realtime Database |

---

## ✨ 주요 기능
1. Multi-Tool Change System
하나의 그리퍼로 다양한 도구를 교체하며 작업합니다.

Pipette: 액체 흡입(Aspiration) 및 분주(Dispensing).

Spoon: 파우더 스쿠핑(Scooping) 및 투입(Pouring).

Stirrer: 믹싱(Stirring).

Tray: 완성된 트레이 이송.

2. 가변 레시피 (Variable Recipe)
6-Hole 트레이의 각 셀마다 서로 다른 배합 비율을 적용할 수 있습니다.

실험계획법(DoE)에 따른 다양한 샘플을 한 번에 제조합니다.

3. 스마트 에러 핸들링 (Smart Error Handling)
일시정지 (Pause/Resume): 작업 중 언제든 웹에서 즉시 정지 및 재개가 가능합니다.

상태 저장 (Checkpoint): 전원이 꺼져도 마지막으로 수행한 세부 동작(Move 단위)부터 정확히 이어서 작업합니다.

반자동 복구 (Semi-Auto Recovery): 충돌 감지 시 즉시 정지하며, 사용자가 웹에서 RESUME을 누르면 자동으로 로봇 리셋 및 서보 온을 수행합니다.

---

## 🔩 하드웨어 구성
Robot: Doosan Robotics M0609 (6축 협동로봇)

End-Effector: Custom Multi-function Gripper (GripperDA_v1)

Tools:

Liquid Pipette & Rack

Powder Spoon & Bowl

Stirring Stick

6-Well Standard Tray

Material: Liquid A/B, Powder A/B

---

## 🚀 설치 및 실행 가이드
1. 사전 요구 사항
ROS2 (Foxy or Humble) installed

Doosan Robotics ROS2 Package installed

Node.js & npm installed

Firebase serviceAccountKey.json 준비

2. 프로젝트 클론
```bash
git clone [https://github.com/your-username/beauro-project.git](https://github.com/your-username/beauro-project.git)
cd beauro-project
```

3. Frontend 실행
```bash
cd frontend
npm install
npm start
# 브라우저에서 http://localhost:3000 접속
```

4. Backend (Monitor) 실행
로봇의 상태를 감시하고 물리적 제어를 담당합니다.
```bash
cd backend
python3 beauro_monitor.py
```
5. Executor (Robot Node) 실행
실제 배합 레시피 로직을 수행합니다.
```bash
cd beauro_node
python3 beauro.py
```

---

## 📂 디렉토리 구조
```bash
beauro-project/
├── frontend/                # React Web Dashboard
│   ├── src/
│   │   ├── components/      # RobotMonitor.tsx 등
│   │   └── lib/             # firebase.ts
│   └── ...
├── backend/                 # System Monitoring Node
│   ├── beauro_monitor.py    # 상태 감시 및 복구 로직
│   └── serviceAccountKey.json
├── beauro_node/             # Robot Operation Node
│   ├── beauro.py            # 메인 공정 로직 (Executor)
│   ├── material_library.yaml # 좌표 및 모션 데이터
│   └── robot_task_state.json # (Local Backup)
└── README.md
```

---

🛡️ License
