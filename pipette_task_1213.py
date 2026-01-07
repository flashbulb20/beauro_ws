import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint, MoveLine, SetCtrlBoxDigitalOutput, GetCtrlBoxDigitalInput
import time

class SyncedPipetteNode(Node):
    def __init__(self):
        super().__init__('synced_pipette_node')

        self.cli_movej = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self.cli_movel = self.create_client(MoveLine, '/dsr01/motion/move_line')
        self.cli_set_out = self.create_client(SetCtrlBoxDigitalOutput, '/dsr01/io/set_ctrl_box_digital_output')
        self.cli_get_in = self.create_client(GetCtrlBoxDigitalInput, '/dsr01/io/get_ctrl_box_digital_input')
        self.wait_for_services()

    def wait_for_services(self):
        while not self.cli_movej.wait_for_service(1.0):
            self.get_logger().info('서비스 연결 대기 중...')
        self.get_logger().info('>>> 로봇 준비 완료!')

    # --- 그리퍼 제어 (대기 시간 강화) ---
    def set_gripper(self, mode):
        # 동작 전 안정화 대기
        time.sleep(0.1)
        
        if mode == 'open': 
            self.send_io(1, 0); self.send_io(2, 0)
            print(f"--- [Gripper] Open  ---")
            time.sleep(0.2) # 충분히 열릴 시간

        elif mode == 'hold': 
            self.send_io(1, 0); self.send_io(2, 1)
            self.wait_input(2) 
            print(f"--- [Gripper] Hold  ---")
            time.sleep(0.2)

        elif mode == 'squeeze': 
            self.send_io(1, 1); self.send_io(2, 1)
            self.wait_input(1) 
            print(f"--- [Gripper] Squeeze  ---")
            time.sleep(0.2)

    def send_io(self, idx, val):
        req = SetCtrlBoxDigitalOutput.Request()
        req.index = idx; req.value = val
        self.cli_set_out.call_async(req)

    def wait_input(self, idx):
        start = time.time()
        while (time.time() - start) < 3.0:
            req = GetCtrlBoxDigitalInput.Request()
            req.index = idx
            future = self.cli_get_in.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result().value == 1: return
            time.sleep(0.1)

    # --- 이동 제어 (완전 정지 보장) ---
    def move_l_stop(self, pos, vel=350.0):
        """이동 후 반드시 멈추고 대기하는 함수"""
        req = MoveLine.Request()
        req.pos = pos
        req.vel = [vel, vel]
        req.acc = [150.0, 150.0]
        
        # [핵심 1] 동기 모드 사용
        req.sync_type = 1 
        
        # [핵심 2] 반경 0.0 -> 정확히 점을 찍고 멈춤 (Blending 없음)
        req.radius = 0.0  
        
        future = self.cli_movel.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        # [핵심 3] 이동 명령이 끝났어도 물리적 진동/통신 딜레이를 위해 잠시 대기
        time.sleep(1.0) 

def main(args=None):
    rclpy.init(args=args)
    node = SyncedPipetteNode()

    # 좌표 설정 (기존과 동일)
    P_PICK   = [170.0, -144.0, 369.55, 118.22, 179.49, 117.84]
    P_WATER  = [432.170, 215.370, 379.720, 26.86, -179.26, 26.06]
    P_TARGET = [425.290, -80.420, 385.770, 10.11, -179.07, 9.54]

    # 안전 높이
    SAFETY_OFFSET = 150.0
    def get_safe_pos(pos):
        return [pos[0], pos[1], pos[2] + SAFETY_OFFSET, pos[3], pos[4], pos[5]]

    P_PICK_UP   = get_safe_pos(P_PICK)
    P_WATER_UP  = get_safe_pos(P_WATER)
    P_TARGET_UP = get_safe_pos(P_TARGET)

    try:
        print(">>> [초기화] 그리퍼 열기")
        node.set_gripper('open')

        # --- 1단계: 스포이드 집기 ---
        print(">>> [Move] 스포이드 위로")
        node.move_l_stop(P_PICK_UP)
        
        print(">>> [Move] 아래로 진입")
        node.move_l_stop(P_PICK)
        
        node.set_gripper('hold')
        
        print(">>> [Move] 위로 탈출")
        node.move_l_stop(P_PICK_UP)

        # --- 2단계: 물 흡입 ---
        print(">>> [Move] 물 컵 위로")
        node.move_l_stop(P_WATER_UP)
        
        print(">>> [Move] 물 속으로 진입")
        node.move_l_stop(P_WATER)

        print("   -> 공기 빼기")
        node.set_gripper('squeeze')
        
        print("   -> 물 흡입")
        node.set_gripper('hold')
        # time.sleep(1.0) # 물 채우기 대기

        print(">>> [Move] 물 밖으로 탈출")
        node.move_l_stop(P_WATER_UP)

        # --- 3단계: 물 배출 ---
        print(">>> [Move] 빈 컵 위로")
        node.move_l_stop(P_TARGET_UP)
        
        print(">>> [Move] 배출 위치로 하강")
        node.move_l_stop(P_TARGET)

        print("   -> 물 짜내기")
        node.set_gripper('squeeze')
        # time.sleep(1.0)
        
        print("   -> 물 짜내기 한번더")
        node.set_gripper('hold')
        node.set_gripper('squeeze')
        # time.sleep(1.0)

        print(">>> [Move] 위로 탈출")
        node.move_l_stop(P_TARGET_UP)

        # --- 4단계: 반납 ---
        print(">>> [Move] 스포이드 반납 위치 위로")
        node.move_l_stop(P_PICK_UP)
        
        print(">>> [Move] 내려놓기")
        node.move_l_stop(P_PICK)
        
        node.set_gripper('open')
        
        print(">>> [Move] 최종 복귀")
        node.move_l_stop(P_PICK_UP)

        print(">>> 작업 완료!")

    except KeyboardInterrupt:
        print("중단됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()