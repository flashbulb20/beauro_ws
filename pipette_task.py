import rclpy
import DR_init
import copy

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도
VELOCITY = 80
VELOCITY_DOWN = 50
VELOCITY_UP = 40
ACC = 80

# 오프셋 설정
Z_OFFSET_APPROACH = 80.0  # 접근 높이

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

def initialize_robot():
    from DSR_ROBOT2 import set_tool, set_tcp
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

def get_offset_pose(pose, z_offset):
    """Z축 높이만 조절한 좌표 반환"""
    new_pose = copy.deepcopy(pose)
    new_pose[2] += z_offset
    return new_pose

def generate_tray_poses(start_pose, rows, cols, x_gap, y_gap):
    """
    기준점(Target 1)에서 시작하여 2x3 그리드 좌표 리스트를 생성하는 함수
    :param start_pose: 1번 구멍의 좌표 (posx)
    :param rows: 행 개수 (여기서는 3)
    :param cols: 열 개수 (여기서는 2)
    :param x_gap: 가로 구멍 간격 (mm)
    :param y_gap: 세로 구멍 간격 (mm)
    :return: 6개의 좌표가 담긴 리스트
    """
    pose_list = []
    
    # 2x3 행렬 순회 (행 우선: 1,2 -> 3,4 -> 5,6 순서)
    for r in range(rows):
        for c in range(cols):
            # 기준 좌표 복사
            current_pose = copy.deepcopy(start_pose)
            
            # X, Y 좌표 계산
            # posx는 [x, y, z, rx, ry, rz] 구조입니다.
            current_pose[0] += (c * x_gap) # 열(Column)이 바뀔 때 X 변경
            current_pose[1] += (r * y_gap) # 행(Row)이 바뀔 때 Y 변경
            
            pose_list.append(current_pose)
            
    return pose_list

def perform_task():
    from DSR_ROBOT2 import movej, movel, posj, posx, set_digital_output, ON, OFF, wait

    HOME_POSE = posj([0, 0, 90, 0, 90, 0])
    ready_to_back_home = posx([323.893, -81.412, 525.051, 19.612, 179.218, 104.354])

    # ---------------------------------------------------------
    # 1. 트레이 설정 (이 부분만 수정하면 6개 구멍 모두 적용됨!)
    # ---------------------------------------------------------
    # Target 1번의 정확한 좌표 (기준점)
    TRAY_REF_POSE = posx([518.147, -176.512, 434.988, 121.573, 179.147, -153.054])
    
    # 자로 측정한 구멍 사이의 간격 (mm)
    # 로봇 좌표계 기준으로 X가 증가하는 방향이면 +, 감소면 -
    TRAY_GAP_X = 57.0   # 오른쪽으로 이동 시 증가한다고 가정
    TRAY_GAP_Y = -37.0  # 몸 쪽으로 당겨질 때 Y값이 줄어든다고 가정 (데이터 기반)

    # 함수를 통해 6개의 좌표 리스트 자동 생성
    tray_targets = generate_tray_poses(TRAY_REF_POSE, rows=3, cols=2, x_gap=TRAY_GAP_X, y_gap=TRAY_GAP_Y)

    # ---------------------------------------------------------
    # 2. 액체/스포이드 데이터
    # ---------------------------------------------------------
    scenarios = {
        1: { # Liquid A
            "spoid_pick": {
                "ready_j": posj([-53.894, -7.946, 97.706, -0.022, 91.036, 30.547]),
                "grip_x":  posx([185.047, -246.289, 371.256, 124.25, 179.082, -151.004]),
            },
            "liquid_cup": posx([585.051, 213.136, 377.027, 149.923, 179.346, -124.974]),
        },
        2: { # Liquid B
            "spoid_pick": {
                "ready_j": posj([-59.006, -2.772, 93.68, -0.022, 89.374, 31.173]),
                "grip_x":  posx([182.36, -295.15, 375.224, 121.088, 179.619, -148.437]),
            },
            "liquid_cup": posx([568.476, -21.046, 380.719, 131.716, 179.174, -142.731]),
        }
    }

    def gripper_init():
        set_digital_output(1, OFF)
        set_digital_output(2, OFF)
    def spoid_hold():
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        wait(1.5)
    def spoid_squeeze():
        set_digital_output(1, ON)
        set_digital_output(2, ON)
        wait(1.5)

    # ---------------------------------------------------------
    # 3. 실행 로직
    # ---------------------------------------------------------
    repeat_count = int(input("How many times?: "))
    liquid_type = int(input("Which liquid (1/2)?: "))
    target_idx = int(input("Where's your target (1~6)?: ")) - 1
    
    current_data = scenarios[liquid_type]
    
    print("Start processing")
    movej(HOME_POSE, vel=VELOCITY, acc=ACC)
    gripper_init()

    # [스포이드 집기]
    pick_grip_x = current_data["spoid_pick"]["grip_x"]
    pick_up_x = get_offset_pose(pick_grip_x, 150.0)
    
    movej(current_data["spoid_pick"]["ready_j"], vel=VELOCITY, acc=ACC)
    movel(pick_grip_x, vel=VELOCITY_DOWN, acc=ACC)
    spoid_hold()
    movel(pick_up_x, vel=VELOCITY_UP, acc=ACC)

    # [피펫팅 루프]
    cup_pos = current_data["liquid_cup"]
    cup_approach = get_offset_pose(cup_pos, Z_OFFSET_APPROACH)
    
    # 계산된 트레이 리스트에서 선택된 타겟 가져오기
    target_pos = tray_targets[target_idx] 
    target_approach = get_offset_pose(target_pos, Z_OFFSET_APPROACH)

    print(f"Target Info: Index {target_idx+1}")
    print(f"Coordinates: {target_pos}") # 디버깅용 좌표 출력

    for i in range(repeat_count):
        print(f"\n[LOOP] {i+1}")
        
        # 흡입
        movel(cup_approach, vel=VELOCITY, acc=ACC)
        movel(cup_pos, vel=VELOCITY_DOWN, acc=ACC)
        spoid_squeeze()
        spoid_hold()
        movel(cup_approach, vel=VELOCITY_UP, acc=ACC)

        # 배출
        movel(target_approach, vel=VELOCITY, acc=ACC)
        movel(target_pos, vel=VELOCITY_DOWN, acc=ACC)
        spoid_squeeze()
        spoid_hold()
        spoid_squeeze()
        spoid_hold()
        movel(target_approach, vel=VELOCITY_UP, acc=ACC)

    # [복귀]
    movel(ready_to_back_home, vel=VELOCITY, acc=ACC)
    movel(pick_up_x, vel=VELOCITY, acc=ACC)
    movel(pick_grip_x, vel=VELOCITY_DOWN, acc=ACC)
    gripper_init()
    movej(current_data["spoid_pick"]["ready_j"], vel=VELOCITY, acc=ACC)
    movej(HOME_POSE, vel=VELOCITY, acc=ACC)
    print("Finished")

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("pipetting_task", namespace=ROBOT_ID)
    DR_init.__dsr__node = node
    try:
        initialize_robot()
        perform_task()
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()