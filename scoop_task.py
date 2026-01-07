import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint, MoveLine, SetCtrlBoxDigitalOutput, GetCtrlBoxDigitalInput
import time

class VerticalSpoonTask(Node):
    def __init__(self):
        super().__init__('vertical_spoon_task')
        self.cli_movej = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self.cli_movel = self.create_client(MoveLine, '/dsr01/motion/move_line')
        self.cli_set_out = self.create_client(SetCtrlBoxDigitalOutput, '/dsr01/io/set_ctrl_box_digital_output')
        self.cli_get_in = self.create_client(GetCtrlBoxDigitalInput, '/dsr01/io/get_ctrl_box_digital_input')
        self.wait_for_services()

    def wait_for_services(self):
        while not self.cli_movej.wait_for_service(1.0):
            self.get_logger().info('서비스 연결 대기 중...')

    def set_gripper(self, mode):
        # 8mm: Close (잡기), 50mm: Open (놓기)
        if mode == 'close':
            self.send_io(1, 1); self.send_io(2, 0)
            self.wait_input_signal(1)
        elif mode == 'open':
            self.send_io(1, 1); self.send_io(2, 1)
            time.sleep(1.0)

    def send_io(self, idx, val):
        req = SetCtrlBoxDigitalOutput.Request()
        req.index = idx; req.value = val
        self.cli_set_out.call_async(req)

    def wait_input_signal(self, idx):
        start = time.time()
        while (time.time() - start) < 3.0:
            req = GetCtrlBoxDigitalInput.Request()
            req.index = idx
            future = self.cli_get_in.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result().value == 1: return True
            time.sleep(0.1)
        return False

    def move_j(self, pos):
        req = MoveJoint.Request()
        req.pos = pos; req.vel = 250.0; req.acc = 120.0; req.sync_type = 1
        future = self.cli_movej.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def move_l(self, pos, vel=250.0, radius=0.0):
        req = MoveLine.Request()
        req.pos = pos; req.vel = [vel, vel]; req.acc = [20.0, 20.0]; req.sync_type = 1
        req.radius = radius
        future = self.cli_movel.call_async(req)
        rclpy.spin_until_future_complete(self, future)

def main(args=None):
    rclpy.init(args=args)
    node = VerticalSpoonTask()

    # ====================================================================
    # [좌표 설정] Home: 367.44, 4.20, 423.14, 169.35, 179.96, 169.67
    # ====================================================================
    
    POS_HOME_J = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

    # [1] 오리엔테이션 정의 (핵심!)
    # ORI_DOWN: 바닥을 봄 (기존 Home 자세) -> B=180
    ORI_DOWN = [169.35, 179.96, 169.67] 
    
    # ORI_FRONT: 정면을 봄 (세로 숟가락 잡기용) -> B=90 으로 변경
    # A(Rz)는 숟가락의 평평한 면 방향에 따라 조절 필요 (여기선 그대로 유지)
    ORI_FRONT = [169.35, 90.00, 169.67]

    # [2] 숟가락 위치 (세로 거치)
    # 숟가락 핸들의 좌표 (Home보다 Y+200, Z는 낮게)
    spoon_x = 367.44
    spoon_y = 200.00
    spoon_z_handle = 250.00 # 핸들 높이

    # 접근: 숟가락보다 X축으로 100mm 뒤에서 접근 (충돌 방지)
    POS_SPOON_READY = [spoon_x - 100.0, spoon_y, spoon_z_handle, ORI_FRONT[0], ORI_FRONT[1], ORI_FRONT[2]]
    POS_SPOON_GRASP = [spoon_x,         spoon_y, spoon_z_handle, ORI_FRONT[0], ORI_FRONT[1], ORI_FRONT[2]]
    
    # 뽑아 올리기 (Z축 상승)
    POS_SPOON_LIFT  = [spoon_x,         spoon_y, spoon_z_handle + 150.0, ORI_FRONT[0], ORI_FRONT[1], ORI_FRONT[2]]

    # [3] 스쿠핑 자세로 변환 (Vertical -> Horizontal)
    # 공중에서 손목을 돌려 숟가락을 눕힘 (ORI_FRONT -> ORI_DOWN)
    POS_CONVERT_MODE = [spoon_x, spoon_y, spoon_z_handle + 150.0, ORI_DOWN[0], ORI_DOWN[1], ORI_DOWN[2]]

    # [4] 파우더 & 컵 위치 (이전과 동일하되 Z높이 주의)
    bowl_pos = [367.44, -150.00, 280.00] 
    cup_pos  = [500.00, -150.00, 350.00]

    try:
        print(">>> 1. 홈 위치 이동")
        node.set_gripper('open')
        node.move_j(POS_HOME_J)

        # ---------------------------------------------------------
        # [Step 1] 세로 숟가락 잡기 (Side Approach)
        # ---------------------------------------------------------
        print(">>> 2. 숟가락 잡기 (수직 접근)")
        
        # 1. 손목을 90도로 꺾어서 접근 위치로 이동
        node.move_l(POS_SPOON_READY, vel=250.0) 
        
        # 2. 앞으로 전진하여 핸들 위치 도달
        node.move_l(POS_SPOON_GRASP, vel=120.0)
        
        # 3. 잡기
        node.set_gripper('close')
        time.sleep(0.5)

        # 4. 위로 쑥 뽑아 올리기
        print("   -> 숟가락 뽑기 (Lift)")
        node.move_l(POS_SPOON_LIFT, vel=250.0)

        # ---------------------------------------------------------
        # [Step 2] 숟가락 눕히기 (Re-orientation)
        # ---------------------------------------------------------
        print(">>> 3. 작업 자세로 변환 (눕히기)")
        # 현재: B=90 (숟가락 서 있음) -> 목표: B=180 (숟가락 누움)
        node.move_l(POS_CONVERT_MODE, vel=120.0)
        time.sleep(0.5)

        # ---------------------------------------------------------
        # [Step 3] 파우더 뜨기 & 붓기 (이전 로직 연결)
        # ---------------------------------------------------------
        print(">>> 4. 파우더 뜨러 이동")
        # 이제 ORI_DOWN(B=180) 상태이므로 이전 코드를 그대로 활용 가능
        
        # 그릇 위로 이동
        pos_bowl_ready = [bowl_pos[0], bowl_pos[1], 350.0, ORI_DOWN[0], ORI_DOWN[1], ORI_DOWN[2]]
        node.move_l(pos_bowl_ready)

        # 스쿠핑 (약식 구현)
        pos_scoop_start = [bowl_pos[0], bowl_pos[1], bowl_pos[2], ORI_DOWN[0], ORI_DOWN[1], ORI_DOWN[2]]
        pos_scoop_end   = [bowl_pos[0]+50, bowl_pos[1], bowl_pos[2], ORI_DOWN[0], 150.0, ORI_DOWN[2]] # Tilt
        
        node.move_l(pos_scoop_start)
        node.move_l(pos_scoop_end, vel=120.0, radius=30.0) # 긁기
        node.move_l(pos_bowl_ready) # 들기

        print(">>> 5. 컵에 붓기")
        # 붓기 자세 (Rx 회전)
        pos_cup_pour = [cup_pos[0], cup_pos[1], 400.0, ORI_DOWN[0], 150.0, ORI_DOWN[2] + 90.0]
        node.move_l(pos_cup_pour, vel=250.0)
        time.sleep(1.0)

        # ---------------------------------------------------------
        # [Step 4] 복귀 및 정리
        # ---------------------------------------------------------
        print(">>> 6. 숟가락 반납 (다시 세우기)")
        # 눕힌 상태 -> 다시 세우기 (B=90)
        node.move_l(POS_CONVERT_MODE) # 공중 대기 위치
        node.move_l(POS_SPOON_LIFT)   # B=90 상태로 변환

        # 꽂아 넣기
        node.move_l(POS_SPOON_GRASP)  # 꽂기
        node.set_gripper('open')      # 놓기
        
        # 뒤로 빠지기
        node.move_l(POS_SPOON_READY)  
        node.move_j(POS_HOME_J)

    except KeyboardInterrupt:
        print("중단됨")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()