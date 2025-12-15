import rclpy
import DR_init
import yaml
import time
import threading

# ==========================================
# 1. 설정 및 전역 변수 (기존과 동일)
# ==========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# [업데이트] 속도 설정 (팀원 코드 반영)
VEL_MOVE = 250
VEL_WORK = 150
ACC = 80

# 트레이 설정
TRAY_PITCH_X = 57.0
TRAY_PITCH_Y = 38.0
TRAY_Z_OFFSET = 0.0

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)
    print(">>> Robot Initialized")

def load_yaml(path):
    with open(path, "r") as f: return yaml.safe_load(f)

def gripper_control(mode):
    from DSR_ROBOT2 import set_digital_output, wait, ON, OFF
    # DI 1, 2는 예시이며 실제 로봇 설정에 따라 변경될 수 있습니다.
    if mode == "init":
        set_digital_output(1, OFF); set_digital_output(2, OFF)
    elif mode == "squeeze":
        set_digital_output(1, ON); set_digital_output(2, ON); wait(1.0)
    elif mode == "hold":
        set_digital_output(1, OFF); set_digital_output(2, ON); wait(1.5)

def get_tray_pose(base_pose, tray_idx):
    from DSR_ROBOT2 import posx
    x, y, z, rx, ry, rz = base_pose[:6]
    idx = tray_idx - 1
    row = idx // 2
    col = idx % 2
    xt = x + (col * TRAY_PITCH_X)
    yt = y - (row * TRAY_PITCH_Y)
    return posx([xt, yt, z + TRAY_Z_OFFSET, rx, ry, rz])

def flatten_and_shake(center_pose):
    from DSR_ROBOT2 import movel, posx
    x, y, z, rx, ry, rz = center_pose
    shake_width = 5
    movel(posx([x, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
    for _ in range(5):
        movel(posx([x+shake_width, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
        movel(posx([x-shake_width, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
    movel(posx([x, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)

def execute_sticks(library, recipe):
    from DSR_ROBOT2 import posx, posj, movel, movej, movec, wait, get_current_posx

    SAFE_Z_OFFSET = 160
    MIXING_ROTATIONS = 3
    NUM_MIXING_WELLS = 6
    
    HOME_POSE = posj([0, 0, 90, 0, 90, 0])
    stick_poses = library["stick"]
    GRAB = posx(stick_poses["grab"]["posx"])
    GRAB_UP = posx(stick_poses["grab_up"]["posx"])
    TRAY_BASE = posx(stick_poses["tray"]["posx"])
    STIR_POSES = posx(stick_poses["stir"]["posx"])
    DROP = posx(stick_poses["drop"]["posx"])

    trays = recipe["trays"] # Dictionary 형태 가정
    for t_idx in trays:
        tray_idx = int(t_idx)

        p_tray = get_tray_pose(TRAY_BASE, tray_idx)
        p_tray_down = posx(list(p_tray))

        # 로봇 초기화
        movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)
        gripper_control("init")

        # 스틱 잡기
        movel(GRAB_UP, vel=VEL_MOVE, acc=ACC)
        movel(GRAB, vel=VEL_WORK, acc=ACC)
        gripper_control("squeeze") 
        movel(GRAB_UP, vel=VEL_MOVE, acc=ACC)
        
        # 트레이 섞기 수행
        for i in range(NUM_MIXING_WELLS):
            well_idx = i + 1

            movel(p_tray, vel=VEL_MOVE, acc=ACC) 
            movel(p_tray_down, vel=VEL_WORK, acc=ACC)
            
            print(f"Well #{well_idx} : {MIXING_ROTATIONS}회 직선 왕복 휘젓기 시작 (총 40mm 왕복)")

            for _ in range(3):
                for p in STIR_POSES:
                    movel(p, vel=VEL_WORK, acc=ACC, radius=10)

            movel(p_tray, vel=VEL_WORK, acc=ACC)

        # 스틱 내려놓기
        movel(DROP, vel=VEL_WORK, acc=ACC)
        movel(posx([DROP[0], DROP[1], DROP[2] - 158] + DROP[3:]))
        gripper_control("init")

        movel(DROP, vel=VEL_WORK, acc=ACC)

    movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("recipe_integration", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        initialize_robot()

        # 경로 수정 필요 (실제 로봇 PC 경로로 변경하세요)
        yaml_path_lib = "/home/hyunjong/beauro_ws/src/robot_control/robot_control/material_library.yaml"
        yaml_path_recipe = "/home/hyunjong/beauro_ws/src/robot_control/robot_control/recipe.yaml"
        
        library = load_yaml(yaml_path_lib)
        recipe = load_yaml(yaml_path_recipe)

        if library and recipe:
            execute_sticks(library, recipe)
            print("\n[SUCCESS] All recipes execution finished.")
        else:
            print("[ERROR] Failed to load YAML files.")

    except KeyboardInterrupt:
        print("\n[STOP] Interrupted by user")
    except Exception as e:
        print(f"\n[ERROR] Unexpected error: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()