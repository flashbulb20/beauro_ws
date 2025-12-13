import rclpy
import DR_init

# 로봇 설정 상수
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

# 이동 속도 및 가속도
# VELOCITY = 100  # simulation
VELOCITY = 80   # real
VELOCITY_DOWN = 50
VELOCITY_UP = 40
ACC = 80

# DR_init 설정
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL


def initialize_robot():
    """로봇의 Tool과 TCP를 설정"""
    from DSR_ROBOT2 import set_tool,set_tcp

    # Tool과 TCP 설정
    set_tool(ROBOT_TOOL)
    set_tcp(ROBOT_TCP)

    # 설정된 설정값 출력
    print("#"*50)
    print("Initializing robot with the following settings:")
    print(f"ROBOT_ID: {ROBOT_ID}")
    print(f"ROBOT_MODEL: {ROBOT_MODEL}")
    print(f"ROBOT_TCP: {ROBOT_TCP}")
    print(f"ROBOT_TOOL: {ROBOT_TOOL}")
    print(f"VELOCITY: {VELOCITY}")
    print(f"ACC: {ACC}")
    print("#"*50)


def perform_task():
    from DSR_ROBOT2 import movej,movel, posj, posx, set_digital_output, ON, OFF, wait

    HOME_POSE = [0, 0, 90, 0, 90, 0]
    spoid_up_point1 = posj([-53.894, -7.946, 97.706, -0.022, 91.036, 30.547])
    spoid_up_point2 = posj([-59.006, -2.772, 93.68, -0.022, 89.374, 31.173])

    spoid_grip_point1 = posx([185.047, -246.289, 371.256, 124.25, 179.082, -151.004])
    spoid_grip_point2 = posx([182.36, -295.15, 375.224, 121.088, 179.619, -148.437])

    spoid_up_from_point1 = posx([185.009, -246.579, 550.862, 124.435, 179.08, -150.168])
    spoid_up_from_point2 = posx([182.189, -294.962, 550.16, 123.933, 179.848, -145.58])

    move_to_liquid1 = posx([583.787, 212.381, 463.135, 140.829, 179.457, -134.162])
    move_to_liquid2 = posx([565.917, -21.93, 492.263, 125.269, 179.22, -149.307])

    move_down_liquid1 = posx([585.051, 213.136, 377.027, 149.923, 179.346, -124.974])
    move_down_liquid2 = posx([568.476, -21.046, 380.719, 131.716, 179.174, -142.731])

    move_up_from_liquid1 = posx([583.787, 212.381, 463.135, 140.829, 179.457, -134.162])
    move_up_from_liquid2 = posx([565.917, -21.93, 492.263, 125.269, 179.22, -149.307])

    move_to_target1l = posx([518.207, -176.499, 483.024, 120.842, 179.159, -153.782])
    move_down_to_target1 = posx([518.147, -176.512, 434.988, 121.573, 179.147, -153.054])
    move_up_from_target1 = posx([518.207, -176.499, 483.024, 120.842, 179.159, -153.782])

    move_to_target2l = posx([574.963, -178.008, 473.841, 92.545, 179.525, 177.804])
    move_down_to_target2 = posx([574.963, -178.008, 423.841, 92.545, 179.525, 177.804])
    move_up_from_target2 = posx([574.963, -178.008, 473.841, 92.545, 179.525, 177.804])

    move_to_target3l = posx([507.878, -213.017, 471.216, 79.707, 179.454, 165.003])
    move_down_to_target3 = posx([507.878, -213.017, 420.216, 79.707, 179.454, 165.003])
    move_up_from_target3 = posx([507.878, -213.017, 471.216, 79.707, 179.454, 165.003])

    move_to_target4l = posx([565.705, -216.192, 467.028, 65.193, 179.546, 150.318])
    move_down_to_target4 = posx([565.705, -216.192, 420.028, 65.193, 179.546, 150.318])
    move_up_from_target4 = posx([565.705, -216.192, 467.028, 65.193, 179.546, 150.318])

    move_to_target5l = posx([504.862, -253.567, 466.186, 46.572, 179.446, 131.775])
    move_down_to_target5 = posx([504.862, -253.567, 420.186, 46.572, 179.446, 131.775])
    move_up_from_target5 = posx([504.862, -253.567, 466.186, 46.572, 179.446, 131.775])

    move_to_target6l = posx([562.78, -253.434, 463.045, 28.426, 179.444, 113.469])
    move_down_to_target6 = posx([562.78, -253.434, 420.045, 28.426, 179.444, 113.469])
    move_up_from_target6 = posx([562.78, -253.434, 463.045, 28.426, 179.444, 113.469])

    ready_to_back_home = posx([323.893, -81.412, 525.051, 19.612, 179.218, 104.354])

    spoid_pick = {
        1: (spoid_up_point1, spoid_grip_point1, spoid_up_from_point1),
        2: (spoid_up_point2, spoid_grip_point2, spoid_up_from_point2),
    }
    liquid_pose = {
        1: (move_to_liquid1, move_down_liquid1, move_up_from_liquid1),
        2: (move_to_liquid2, move_down_liquid2, move_up_from_liquid2),
    }
    target_pose = {
        1: (move_to_target1l, move_down_to_target1, move_up_from_target1),
        2: (move_to_target2l, move_down_to_target2, move_up_from_target2),
        3: (move_to_target3l, move_down_to_target3, move_up_from_target3),
        4: (move_to_target4l, move_down_to_target4, move_up_from_target4),
        5: (move_to_target5l, move_down_to_target5, move_up_from_target5),
        6: (move_to_target6l, move_down_to_target6, move_up_from_target6),
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

    repeat_count = int(input("How many time?: "))
    liquid_type = int(input("What liquid you want?(1/2): "))
    spoid_num = liquid_type
    target_num = int(input("Where's your target?(1~6): "))
    
    to_liq, down_liq, up_liq,  = liquid_pose[liquid_type]
    tl, tdown, tup = target_pose[target_num]
    up_j, grip_x, up_from_x = spoid_pick[spoid_num]

    print("Start processing")
    movej(HOME_POSE, vel=VELOCITY, acc=ACC)
    gripper_init()

    print(f"pick spoid{spoid_num}")
    movej(up_j, vel=VELOCITY, acc=ACC)
    movel(grip_x, vel=VELOCITY_DOWN, acc=ACC)
    spoid_hold()
    movel(up_from_x, vel=VELOCITY_UP, acc=ACC)

    for i in range(repeat_count):
        print(f"\n[LOOP] {i+1}")
        movel(to_liq, vel=VELOCITY, acc=ACC)
        movel(down_liq, vel=VELOCITY_DOWN, acc=ACC)
        spoid_squeeze()
        spoid_hold()
        movel(up_liq, vel=VELOCITY_UP, acc=ACC)
        movel(tl, vel=VELOCITY, acc=ACC)
        movel(tdown, vel=VELOCITY_DOWN, acc=ACC)
        spoid_squeeze()
        spoid_hold()
        spoid_squeeze()
        spoid_hold()
        movel(tup, vel=VELOCITY_UP, acc=ACC)

    movel(ready_to_back_home, vel=VELOCITY, acc=ACC)

    movel(up_from_x, vel=VELOCITY, acc=ACC)
    movel(grip_x, vel=VELOCITY_DOWN, acc=ACC)
    gripper_init()
    movej(up_j, vel=VELOCITY, acc=ACC)
    movej(HOME_POSE, vel=VELOCITY, acc=ACC) # 얘는 통합할 때 없앨지 유지할지 고려
    print("Finished processing")



def main(args=None):
    """메인 함수: ROS2 노드 초기화 및 동작 수행"""
    rclpy.init(args=args)
    node = rclpy.create_node("go_home", namespace=ROBOT_ID)

    # DR_init에 노드 설정
    DR_init.__dsr__node = node

    try:
        # 초기화는 한 번만 수행
        initialize_robot()
        # 작업 수행
        perform_task()

    except KeyboardInterrupt:
        print("\nNode interrupted by user. Shutting down...")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
