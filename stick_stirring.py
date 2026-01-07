import rclpy
import DR_init
import copy
import yaml
import time

# ==========================================
# 1. 설정 및 상수
# ==========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

VEL_MOVE = 150
VEL_WORK = 100
ACC = 80
CORNER_WAIT = 0.5 # 모서리 정지 시간

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)

def get_target_up_pose(target_xy_pose, reference_up_pose):
    """목표 XY 위치에 안전 높이(reference Z)를 적용"""
    new_pose = copy.deepcopy(target_xy_pose)
    new_pose[2] = reference_up_pose[2] 
    return new_pose

def perform_task_from_yaml(yaml_data):
    from DSR_ROBOT2 import movej, movel, posj, posx, set_digital_output, wait, ON, OFF

    # ---------------------------------------------------------
    # 그리퍼 제어 함수
    # ---------------------------------------------------------
    def gripper_open():
        set_digital_output(1, OFF); set_digital_output(2, OFF); wait(0.5)

    def gripper_grab():
        set_digital_output(1, ON); set_digital_output(2, ON); wait(1.0)

    # ---------------------------------------------------------
    # 초기화
    # ---------------------------------------------------------
    HOME_POSE = posj([0, 0, 90, 0, 90, 0])
    movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)
    gripper_open()

    print(">>> Start Task using YAML Coordinates (Strict Square Path)")

    # ---------------------------------------------------------
    # YAML 데이터 순회 (stick_1 -> stick_6)
    # ---------------------------------------------------------
    # 키 순서 보장을 위해 정렬 (stick_1, stick_10 등 방지용으로 길이/알파벳 정렬 고려 가능하나 여기선 단순 정렬)
    stick_keys = sorted(yaml_data.keys()) 

    for key in stick_keys:
        if not key.startswith("stick"): continue # stick으로 시작하는 것만 실행
        
        data = yaml_data[key]
        print(f"\n[{key}] Processing...")

        # 좌표 로드
        GRAB_DOWN = posx(data["grab"]["posx"])
        GRAB_UP   = posx(data["grab_up"]["posx"]) # 이것이 안전 높이(Ceiling) 기준이 됩니다.
        STIR_PATH = [posx(p) for p in data["stir"]["posx"]]
        DROP_DOWN = posx(data["drop"]["posx"])

        # =========================================================
        # [1] 막대기 집기 (Home/Drop -> GRAB_UP -> GRAB_DOWN)
        # =========================================================
        # 1. 안전 높이로 이동 (이전 위치가 어디든 상관없이)
        movel(GRAB_UP, vel=VEL_MOVE, acc=ACC)
        wait(CORNER_WAIT)

        # 2. 하강 및 집기
        movel(GRAB_DOWN, vel=VEL_WORK, acc=ACC)
        gripper_grab()

        # 3. 상승
        movel(GRAB_UP, vel=VEL_MOVE, acc=ACC)
        wait(CORNER_WAIT)


        # =========================================================
        # [2] 트레이 젓기 (Stick -> Stir Start)
        # =========================================================
        # YAML에 있는 Stir 좌표는 모두 '바닥(Down)' 좌표입니다.
        # 안전한 진입을 위해 첫 번째 점의 '상공(Up)' 좌표를 만들어야 합니다.
        
        stir_start_down = STIR_PATH[0]
        stir_end_down   = STIR_PATH[-1]
        
        stir_start_up = get_target_up_pose(stir_start_down, GRAB_UP)
        stir_end_up   = get_target_up_pose(stir_end_down, GRAB_UP)

        # 1. 젓기 시작점 상공으로 수평 이동
        movel(stir_start_up, vel=VEL_MOVE, acc=ACC)
        wait(CORNER_WAIT)

        # 2. 젓기 시작점으로 하강
        movel(stir_start_down, vel=VEL_WORK, acc=ACC)

        # 3. 젓기 수행
        for _ in range(3):
            for idx, point in enumerate(STIR_PATH):
                # 마지막 포인트는 정지 (radius 제거)
                if idx == len(STIR_PATH) - 1:
                    movel(point, vel=VEL_WORK, acc=ACC)
                else:
                    movel(point, vel=VEL_WORK, acc=ACC, radius=20)
        
        wait(0.2)

        # 4. 젓기 끝난 지점에서 수직 상승 (탈출)
        movel(stir_end_up, vel=VEL_MOVE, acc=ACC)
        wait(CORNER_WAIT)


        # =========================================================
        # [3] 버리기 (Tray -> Drop)
        # =========================================================
        drop_up = get_target_up_pose(DROP_DOWN, GRAB_UP)

        # 1. Drop 상공 이동
        movel(drop_up, vel=VEL_MOVE, acc=ACC)
        wait(CORNER_WAIT)

        # 2. 하강 및 버리기
        movel(DROP_DOWN, vel=VEL_WORK, acc=ACC)
        gripper_open()

        # 3. 상승
        movel(drop_up, vel=VEL_MOVE, acc=ACC)
        wait(CORNER_WAIT)

    # 종료 후 홈 복귀
    movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)
    print(">>> All tasks finished.")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("yaml_stir_task", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    # YAML 파일 경로 설정 (사용자 환경에 맞게 수정 필요)
    # 예: yaml_path = "/home/user/catkin_ws/src/pkg/config/material_library.yaml"
    yaml_path = "/home/flashbulb/beauro_ws/src/beauro_project/beauro_project/material_library.yaml"

    try:
        initialize_robot()
        yaml_data = load_yaml(yaml_path)
        if yaml_data:
            perform_task_from_yaml(yaml_data)
        else:
            print("Failed to load YAML")
            
    except KeyboardInterrupt:
        print("\nStopped.")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()