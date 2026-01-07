import rclpy
import DR_init
import yaml
import time
import os
import firebase_admin
from firebase_admin import credentials, db

# ==========================================
# 1. 설정 및 상수
# ==========================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 속도 설정
VEL_MOVE = 2000
VEL_WORK = 2000
ACC = 1000

# 트레이 설정
TRAY_PITCH_X = 57.0
TRAY_PITCH_Y = 38.0

# 경로 설정
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
FIREBASE_KEY_PATH = os.path.join(BASE_DIR, "serviceAccountKey.json")
LIBRARY_YAML_PATH = os.path.join(BASE_DIR, "material_library.yaml")
FIREBASE_DB_URL = 'https://beauro-ac0ad-default-rtdb.asia-southeast1.firebasedatabase.app/'

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# ==========================================
# 2. 유틸리티 함수
# ==========================================
def load_yaml(path):
    try:
        with open(path, "r") as f: return yaml.safe_load(f)
    except Exception as e:
        print(f"[ERROR] YAML Load Failed: {path}\n{e}")
        return None

def get_pose_value(data_dict, key):
    target = data_dict.get(key)
    if not target:
        print(f"[WARN] Key '{key}' not found.")
        return None
    if "posx" in target: return target["posx"]
    if "posj" in target: return target["posj"]
    if "value" in target: return target["value"]
    if isinstance(target, list): return target
    return None

def update_status(action, progress, well_id=0, state_code=2):
    print(f" -> [Action] {action} (Well: {well_id}, {int(progress)}%)")
    try:
        db.reference('robot_state').update({
            'state': state_code,
            'current_action': action,
            'progress_percent': int(progress),
            'current_well': well_id,
            'timestamp': int(time.time() * 1000)
        })
    except: pass

def get_tray_pose(base_pose, tray_idx):
    from DSR_ROBOT2 import posx
    x, y, z, rx, ry, rz = base_pose[:6]
    idx = tray_idx - 1
    row = idx // 2; col = idx % 2
    # YAML 데이터 기준: X+60, Y-40
    xt = x + (col * 60.0)
    yt = y - (row * 40.0)
    return posx([xt, yt, z, rx, ry, rz])

def make_stir_pose(base_stir, p_tray_down, TRAY_BASE):
    from DSR_ROBOT2 import posx
    dx = p_tray_down[0] - TRAY_BASE[0]
    dy = p_tray_down[1] - TRAY_BASE[1]
    return posx([base_stir[0] + dx, base_stir[1] + dy, p_tray_down[2], base_stir[3], base_stir[4], base_stir[5]])

def flatten_and_shake(center_pose):
    from DSR_ROBOT2 import movel, posx
    x, y, z, rx, ry, rz = center_pose
    shake_width = 5.0
    movel(posx([x, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
    for _ in range(3):
        movel(posx([x+shake_width, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
        movel(posx([x-shake_width, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)
    movel(posx([x, y, z, rx, ry, rz]), vel=VEL_MOVE, acc=ACC)

def gripper_control(mode):
    from DSR_ROBOT2 import set_digital_output, wait, ON, OFF
    if mode == "init": set_digital_output(1, OFF); set_digital_output(2, OFF)
    elif mode == "squeeze": set_digital_output(1, ON); set_digital_output(2, ON); wait(1.0)
    elif mode == "hold": set_digital_output(1, OFF); set_digital_output(2, ON); wait(1.5)

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TOOL); set_tcp(ROBOT_TCP)
    print(">>> Robot Initialized")

# ==========================================
# 3. 공정 함수
# ==========================================

def execute_liquid(library, recipe, current_step, total_steps):
    from DSR_ROBOT2 import posx, posj, movel, movej

    # 현재 레시피에 액체가 없으면 스킵
    if not recipe["selection"].get("liquid"): return current_step

    liquid_key = recipe["selection"]["liquid"]
    liq_data = library["liquids"][liquid_key]["poses"]
    
    p_grab_up = posx(get_pose_value(liq_data, "grab_up"))
    p_grab    = posx(get_pose_value(liq_data, "grab"))
    
    if "cup_up" in liq_data:
        p_cup_up = posx(get_pose_value(liq_data, "cup_up"))
        p_cup_down = posx(get_pose_value(liq_data, "cup_down"))
    else:
        # liquid_B 구조 대응 (cup, cup_up 혼용 시)
        p_cup_up = posx(get_pose_value(liq_data, "cup"))
        p_cup_down = posx(list(p_cup_up)); p_cup_down[2] -= 150 # 임의 깊이

    tray_data = liq_data["trays"]
    if "values" in tray_data: tray_base_raw = tray_data["values"][0]
    else: tray_base_raw = get_pose_value(liq_data, "trays")

    progress = (current_step / total_steps) * 100
    update_status(f"Setting up {liquid_key}", progress)

    movej(posj([0, 0, 90, 0, 90, 0]), vel=VEL_MOVE, acc=ACC)
    gripper_control("init")

    update_status(f"Picking Pipette ({liquid_key})", progress)
    movel(p_grab_up, vel=VEL_MOVE, acc=ACC)
    movel(p_grab, vel=VEL_WORK, acc=ACC)
    gripper_control("hold")
    movel(p_grab_up, vel=VEL_MOVE, acc=ACC)

    trays = recipe["trays"]
    for t_idx, t_cfg in trays.items():
        count = t_cfg["count"].get("liquid", 0)
        if count <= 0: continue
        
        tray_idx = int(t_idx)
        current_step += 1
        progress = (current_step / total_steps) * 100
        update_status(f"Dispensing {liquid_key} to Well {tray_idx}", progress, tray_idx)
        
        p_tray = get_tray_pose(tray_base_raw, tray_idx)
        p_tray_up = posx(list(p_tray))
        p_tray_up[2] += 50.0

        for c in range(count):
            movel(p_cup_up, vel=VEL_MOVE, acc=ACC)
            gripper_control("squeeze")
            movel(p_cup_down, vel=VEL_WORK, acc=ACC)
            gripper_control("hold")
            movel(p_cup_up, vel=VEL_MOVE, acc=ACC)

            movel(p_tray_up, vel=VEL_MOVE, acc=ACC)
            movel(p_tray, vel=VEL_WORK, acc=ACC)
            gripper_control("squeeze"); gripper_control("hold")
            gripper_control("squeeze"); gripper_control("hold")
            movel(p_tray_up, vel=VEL_MOVE, acc=ACC)

    update_status("Returning Pipette", progress)
    movel(p_grab_up, vel=VEL_MOVE, acc=ACC)
    movel(p_grab, vel=VEL_WORK, acc=ACC)
    gripper_control("init")
    movel(p_grab_up, vel=VEL_MOVE, acc=ACC)
    
    return current_step

def execute_powder(library, recipe, current_step, total_steps):
    from DSR_ROBOT2 import posx, posj, movel, movej

    if not recipe["selection"].get("powder"): return current_step

    powder_key = recipe["selection"]["powder"]
    pow_data = library["powders"][powder_key]["poses"]

    p_grab = posx(get_pose_value(pow_data, "grab"))
    xg, yg, zg, rxg, ryg, rzg = p_grab

    p_bowl = posx(get_pose_value(pow_data, "bowl"))
    p_scoop_1 = posj(get_pose_value(pow_data, "scoop_1"))
    p_scoop_2 = posj(get_pose_value(pow_data, "scoop_2"))
    p_scoop_3 = posj(get_pose_value(pow_data, "scoop_3"))
    p_flat = posx(get_pose_value(pow_data, "flat"))
    p_tray_base = get_pose_value(pow_data, "tray_base")
    p_pour_list = get_pose_value(pow_data, "pour")

    progress = (current_step / total_steps) * 100
    update_status(f"Setting up {powder_key}", progress)

    gripper_control("init")
    update_status(f"Picking Spoon ({powder_key})", progress)
    
    movel(posx([xg, yg, zg+80, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
    movel(p_grab, vel=VEL_WORK, acc=ACC)
    gripper_control("squeeze")

    spoon_shift = -40 if powder_key == "powder_A" else 40
    movel(posx([xg + spoon_shift, yg, zg, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
    movel(posx([xg + spoon_shift, yg, zg+110, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)

    trays = recipe["trays"]
    for t_idx, t_cfg in trays.items():
        count = t_cfg["count"].get("powder", 0)
        if count <= 0: continue
        
        tray_idx = int(t_idx)
        current_step += 1
        progress = (current_step / total_steps) * 100
        update_status(f"Dispensing {powder_key} to Well {tray_idx}", progress, tray_idx)

        p_tray = get_tray_pose(p_tray_base, tray_idx)
        
        if p_pour_list and len(p_pour_list) >= tray_idx:
            p_pour = posj(p_pour_list[tray_idx - 1])
        else:
            continue

        for c in range(count):
            movel(p_bowl, vel=VEL_MOVE, acc=ACC) 
            movej(p_scoop_1, vel=VEL_WORK, acc=ACC)
            movej(p_scoop_2, vel=VEL_WORK, acc=ACC)
            movej(p_scoop_3, vel=VEL_WORK, acc=ACC)
            
            movel(p_flat, vel=VEL_MOVE, acc=ACC)
            flatten_and_shake(p_flat)

            movel(p_tray, vel=VEL_MOVE, acc=ACC)
            movej(p_pour, vel=VEL_MOVE, acc=ACC)
            
            POUR_ANGLE = -90 if powder_key == "powder_A" else 90
            j1, j2, j3, j4, j5, j6 = p_pour
            p_pour_j = posj([j1, j2, j3, j4, j5, j6+POUR_ANGLE])
            movej(p_pour_j, vel=VEL_WORK, acc=ACC)
            
            for _ in range(3):
                movej(posj([j1, j2, j3, j4, j5, j6+POUR_ANGLE + 5.0]), vel=VEL_WORK, acc=ACC)
                movej(posj([j1, j2, j3, j4, j5, j6+POUR_ANGLE - 5.0]), vel=VEL_WORK, acc=ACC)
            
            movel(p_tray, vel=VEL_MOVE, acc=ACC)

    update_status("Returning Spoon", progress)
    movel(posx([xg + spoon_shift, yg, zg+80, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
    movel(posx([xg + spoon_shift, yg, zg, rxg, ryg, rzg]), vel=VEL_MOVE, acc=ACC)
    movel(p_grab, vel=VEL_WORK, acc=ACC)
    gripper_control("init")
    movej(posj([0, 0, 90, 0, 90, 0]), vel=VEL_MOVE, acc=ACC)
    
    return current_step

def execute_sticks(library, recipe, current_step, total_steps):
    from DSR_ROBOT2 import posx, posj, movel, movej
    
    progress = (current_step / total_steps) * 100
    update_status("Starting Mixing", progress)
    
    HOME_POSE = posj([0, 0, 90, 0, 90, 0])
    stick_poses = library["stick"]
    
    GRAB = posx(get_pose_value(stick_poses, "grab"))
    GRAB_UP = posx(get_pose_value(stick_poses, "grab_up"))
    TRAY_BASE = posx(get_pose_value(stick_poses, "tray"))
    STIR_POSES_BASE = [posx(p) for p in get_pose_value(stick_poses, "stir")]
    DROP = posx(get_pose_value(stick_poses, "drop"))
    
    TRAY_UP_Z, TRAY_DOWN_Z = 550, 427
    trays = recipe["trays"]

    movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)

    for t_idx in trays:
        # 뭐라도 담긴 웰만 믹싱 수행
        tray_idx = int(t_idx)
        well_counts = trays[t_idx]["count"]
        if (well_counts.get("liquid", 0) + well_counts.get("powder", 0)) == 0:
            continue

        current_step += 1
        progress = (current_step / total_steps) * 100
        update_status(f"Mixing Well {tray_idx}", progress, tray_idx)

        gripper_control("init")
        movel(GRAB_UP, vel=VEL_MOVE, acc=ACC)
        movel(GRAB, vel=VEL_WORK, acc=ACC)
        gripper_control("squeeze")
        movel(GRAB_UP, vel=VEL_MOVE, acc=ACC)

        p_tray = get_tray_pose(TRAY_BASE, tray_idx)
        p_tray_up = posx([p_tray[0], p_tray[1], TRAY_UP_Z, p_tray[3], p_tray[4], p_tray[5]])
        p_tray_down = posx([p_tray[0], p_tray[1], TRAY_DOWN_Z, p_tray[3], p_tray[4], p_tray[5]])

        stir_poses_tray = [make_stir_pose(p, p_tray_down, TRAY_BASE) for p in STIR_POSES_BASE]

        movel(p_tray_up, vel=VEL_MOVE, acc=ACC)
        movel(p_tray_down, vel=VEL_WORK, acc=ACC)

        for _ in range(3):
            for i, p in enumerate(stir_poses_tray):
                rad = 0 if i == len(stir_poses_tray) - 1 else 10
                movel(p, vel=VEL_WORK, acc=ACC, radius=rad)

        movel(p_tray_up, vel=VEL_MOVE, acc=ACC)
        movel(DROP, vel=VEL_MOVE, acc=ACC)
        movel(posx([DROP[0], DROP[1], DROP[2] - 158, DROP[3], DROP[4], DROP[5]]), vel=VEL_WORK, acc=ACC)

        gripper_control("init")
        movel(DROP, vel=VEL_MOVE, acc=ACC)

    movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)
    return current_step

def execute_tray(library):
    from DSR_ROBOT2 import posx, posj, movel, movej
    update_status("Transferring Tray to Output", 95)

    HOME_POSE = posj([0, 0, 90, 0, 90, 0])
    stick_poses = library["tray_out"]

    READY_1 = posx(get_pose_value(stick_poses, "ready_1"))
    READY_2 = posx(get_pose_value(stick_poses, "ready_2"))
    GRAB = posx(get_pose_value(stick_poses, "grab"))
    DROP = posx(get_pose_value(stick_poses, "drop"))
    FINISH = posx(get_pose_value(stick_poses, "finished"))

    gripper_control("init")
    movel(READY_1, vel=VEL_MOVE, acc=ACC)
    movel(READY_2, vel=VEL_MOVE, acc=ACC)
    movel(GRAB, vel=VEL_MOVE, acc=ACC)
    gripper_control("hold")

    movel(posx(GRAB[0:2] + [GRAB[2] + 100] + GRAB[3:6]), vel=VEL_MOVE, acc=ACC)
    movel(DROP, vel=VEL_MOVE, acc=ACC)
    gripper_control("init")

    movel(FINISH, vel=VEL_MOVE, acc=ACC)
    movel(posx(FINISH[0:2] + [FINISH[2] + 80] + FINISH[3:6]), vel=VEL_MOVE, acc=ACC)
    movej(HOME_POSE, vel=VEL_MOVE, acc=ACC)

# ==========================================
# 4. 재료별 레시피 생성 헬퍼
# ==========================================
def create_dynamic_recipe_for_ingredient(doe_matrix, target_code, yaml_key):
    """
    특정 재료 코드(예: 'l1')만 포함하는 임시 레시피를 생성합니다.
    """
    trays_config = {}
    has_work = False
    
    # target_code가 액체인지 파우더인지 판단
    is_liquid = target_code.startswith('l')
    
    for well in doe_matrix:
        w_id = well['well_id']
        counts = well.get('counts', {})
        count = counts.get(target_code, 0)
        
        if count > 0:
            has_work = True
            trays_config[w_id] = {
                "count": { 
                    "liquid": count if is_liquid else 0,
                    "powder": count if not is_liquid else 0
                }
            }
            
    if not has_work:
        return None

    return {
        "selection": { 
            "liquid": yaml_key if is_liquid else None, 
            "powder": yaml_key if not is_liquid else None 
        },
        "trays": trays_config
    }

# ==========================================
# 5. 메인 함수
# ==========================================
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("beauro_executor", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    print("🔑 Initializing Firebase...")
    try:
        cred = credentials.Certificate(FIREBASE_KEY_PATH)
        firebase_admin.initialize_app(cred, {'databaseURL': FIREBASE_DB_URL})
        print("✅ Connected.")
    except Exception as e:
        print(f"❌ Init Error: {e}"); return

    initialize_robot()

    library = load_yaml(LIBRARY_YAML_PATH)
    if not library: return

    ref_order = db.reference('current_order')

    print("\n✅ Executor Ready. Waiting for Web Order...")

    try:
        while rclpy.ok():
            order = ref_order.get()
            
            if order and order.get('status') == 'start':
                print(f"\n🔔 Processing Order: {order.get('order_id')}")
                ref_order.update({'status': 'processing'})
                
                # ---------------------------------------------------------
                # [Phase 1] 분주 (Dispensing)
                # ---------------------------------------------------------
                update_status("Initializing...", 0, 0, 2)

                doe_matrix = order.get('doe_matrix', [])
                
                dispense_ops = 0
                for well in doe_matrix:
                    c = well.get('counts', {})
                    dispense_ops += c.get('l1', 0) + c.get('l2', 0)
                    dispense_ops += c.get('p1', 0) + c.get('p2', 0)
                
                total_dispense_steps = dispense_ops + 5
                current_step = 0

                trays_config = {}
                for well in doe_matrix:
                    w_id = well['well_id']
                    counts = well.get('counts', {})
                    l_count = counts.get('l1', 0) + counts.get('l2', 0)
                    p_count = counts.get('p1', 0) + counts.get('p2', 0)
                    trays_config[w_id] = {"count": { "liquid": l_count, "powder": p_count }}
                
                # 재료별 수행 (L1 -> L2 -> P1 -> P2)
                recipe_l1 = create_dynamic_recipe_for_ingredient(doe_matrix, 'l1', 'liquid_A')
                if recipe_l1:
                    current_step = execute_liquid(library, recipe_l1, current_step, total_dispense_steps)
                    
                recipe_l2 = create_dynamic_recipe_for_ingredient(doe_matrix, 'l2', 'liquid_B')
                if recipe_l2:
                    current_step = execute_liquid(library, recipe_l2, current_step, total_dispense_steps)
                    
                recipe_p1 = create_dynamic_recipe_for_ingredient(doe_matrix, 'p1', 'powder_A')
                if recipe_p1:
                    current_step = execute_powder(library, recipe_p1, current_step, total_dispense_steps)
                    
                recipe_p2 = create_dynamic_recipe_for_ingredient(doe_matrix, 'p2', 'powder_B')
                if recipe_p2:
                    current_step = execute_powder(library, recipe_p2, current_step, total_dispense_steps)

                print(">> Dispensing Phase Done (100%)")
                update_status("Dispensing Done. Switch to Mixer...", 100, 0, 2)
                
                time.sleep(3.0) 

                # ---------------------------------------------------------
                # [Phase 2] 믹싱 & 배출 (Mixing & Output)
                # ---------------------------------------------------------
                
                mix_well_count = 0
                mix_trays = {}
                for well in doe_matrix:
                    c = well.get('counts', {})
                    if sum(c.values()) > 0:
                        mix_well_count += 1
                        mix_trays[well['well_id']] = {"count": {"liquid": 1, "powder": 1}}

                mix_recipe = {"trays": mix_trays}
                total_mix_steps = mix_well_count + 2
                current_mix_step = 0
                
                update_status("Starting Mixing Process...", 0, 0, 2)

                # Mixing 실행
                current_mix_step = execute_sticks(library, mix_recipe, current_mix_step, total_mix_steps)

                execute_tray(library)

                print(">> All Actions Done. Sending 100%...")
                update_status("Output Process Completed.", 100, 0, 2)
                
                time.sleep(2.0)

                ref_order.update({'status': 'finished'})
                print("✨ Order Status -> finished")
                
            else:
                time.sleep(1)

    except KeyboardInterrupt:
        print("\n[STOP]")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()