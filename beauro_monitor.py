import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
import firebase_admin
from firebase_admin import credentials, db
import time
import math
import threading

# 서비스 및 토픽 메시지
# [추가] SetRobotControl 추가됨
from dsr_msgs2.srv import GetRobotState, GetRobotMode, GetCurrentPosx, MoveJoint, MoveLine, SetRobotControl
from sensor_msgs.msg import JointState

# 상태 매핑
ROBOT_STATE_MAP = {
    0: "STATE_UNKNOWN", 1: "STATE_INITIALIZING", 2: "STATE_STANDBY",
    3: "STATE_MOVING", 4: "STATE_SAFE_OFF", 5: "STATE_TEACHING",
    6: "STATE_SAFE_STOP", 7: "STATE_EMERGENCY_STOP", 8: "STATE_HOMING", 9: "STATE_RECOVERY"
}

ERROR_STATE_CODES = [4, 6, 7, 9]

class BeauroFullSystem(Node):
    def __init__(self):
        super().__init__('beauro_full_system')
        self.get_logger().info("🚀 Beauro Monitor: Monitoring & Recovery System Started")

        # 1. Firebase 접속
        try:
            if not firebase_admin._apps:
                cred = credentials.Certificate("serviceAccountKey.json")
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://beauro-ac0ad-default-rtdb.asia-southeast1.firebasedatabase.app/' 
                })
            self.ref_status = db.reference('robot_state')
            self.ref_command = db.reference('manual_command')
            self.ref_recovery = db.reference('command/recovery') # [추가] 복구 명령 참조
            self.get_logger().info("✅ Firebase Connected")
        except Exception as e:
            self.get_logger().error(f"Firebase Error: {e}")

        # 2. 데이터 저장소
        self.robot_data = {
            'is_online': False, 'state_code': 0, 'status': "DISCONNECTED",
            'mode': 0, 'tcp': {'x':0,'y':0,'z':0,'a':0,'b':0,'c':0}, 
            'joint': [0]*6, 'timestamp': 0
        }
        self.last_alive_time = 0 

        # 3. [Monitor] 클라이언트 & 구독 설정
        qos_profile = QoSProfile(depth=10)
        self.create_subscription(JointState, '/dsr01/joint_states', self.topic_callback_joint, qos_profile)
        
        self.cli_state = self.create_client(GetRobotState, '/dsr01/system/get_robot_state')
        self.cli_mode = self.create_client(GetRobotMode, '/dsr01/system/get_robot_mode')
        self.cli_tcp = self.create_client(GetCurrentPosx, '/dsr01/aux_control/get_current_posx')

        # 4. [Control] 이동 및 제어 서비스
        self.cli_movej = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self.cli_movel = self.create_client(MoveLine, '/dsr01/motion/move_line')
        
        # [추가] 로봇 상태 제어 서비스 (리셋, 서보온 등)
        self.cli_control = self.create_client(SetRobotControl, '/dsr01/system/set_robot_control')

        # 5. [Listener] 리스너 스레드 시작
        self.listener_thread = threading.Thread(target=self.start_listeners)
        self.listener_thread.daemon = True
        self.listener_thread.start()

        # 6. [Monitor] 주기적 업데이트 타이머 (0.5초)
        self.timer = self.create_timer(0.5, self.routine_check)

    # ------------------------------------------------------------------
    # Part 1: Listeners (Manual Move + Recovery)
    # ------------------------------------------------------------------
    def start_listeners(self):
        """Firebase 리스너들을 등록"""
        # 수동 이동 명령 감시
        self.ref_command.listen(self.on_command_received)
        # 복구 명령 감시 (Resume/Stop)
        self.ref_recovery.listen(self.on_recovery_received)

    def on_command_received(self, event):
        """수동 이동 명령 처리"""
        if event.data is None: return
        cmd = self.ref_command.get()
        if not cmd: return

        self.get_logger().info(f"📩 Move Command: {cmd.get('type')}")
        try:
            if cmd['type'] == 'move_line':
                target = cmd['target']
                pos_list = [float(target['x']), float(target['y']), float(target['z']),
                            float(target['a']), float(target['b']), float(target['c'])]
                self.execute_move_line(pos_list)
            elif cmd['type'] == 'move_joint':
                pos_list = [float(x) for x in cmd['target']]
                self.execute_move_joint(pos_list)
            self.ref_command.delete()
        except Exception as e:
            self.get_logger().error(f"Move Error: {e}")

    # [신규] 복구 명령 처리 함수
    def on_recovery_received(self, event):
        """웹에서 Resume/Stop 버튼을 눌렀을 때 처리"""
        if event.data is None: return
        
        # event.data가 딕셔너리가 아닌 문자열 값일 수 있음 ('resume' or 'stop')
        cmd = event.data
        # 만약 path가 하위 경로라면 전체 데이터를 다시 가져옴
        if event.path != '/': 
            cmd = self.ref_recovery.get()

        if cmd == 'resume':
            self.get_logger().info("🚑 Recovery Command Received: RESUME")
            self.execute_auto_recovery()
            # 처리가 끝나면 명령 삭제 (Executor와 충돌 방지 위해 딜레이 없이 삭제하지 않거나 상황에 맞춰야 함)
            # 여기서는 Monitor가 주도적으로 복구하고 삭제함
            self.ref_recovery.delete()
            
        elif cmd == 'stop':
            self.get_logger().info("🛑 Recovery Command Received: STOP")
            # Stop 처리는 Executor가 루프를 멈추는 것이므로 여기서는 특별한 하드웨어 동작 없음
            # 다만 로봇을 멈추기 위해 Stop 동작을 수행할 수도 있음
            self.ref_recovery.delete()

    # ------------------------------------------------------------------
    # Part 2: Execution & Recovery Logic
    # ------------------------------------------------------------------
    def execute_auto_recovery(self):
        """자동 복구 시퀀스 실행 (SafeStop Reset -> Servo On)"""
        if not self.cli_control.service_is_ready():
            self.get_logger().warn("⚠️ Control Service Not Ready")
            return

        self.get_logger().info("🔄 Starting Recovery Sequence...")
        
        # 1. Reset (Fault Clear)
        self.call_control_service(3) 
        time.sleep(1.0)
        
        # 2. Safe Stop Reset
        self.call_control_service(2)
        time.sleep(1.0)
        
        # 3. Servo On
        self.call_control_service(1)
        time.sleep(2.0)
        
        # 4. 상태 업데이트 (에러 해제 알림)
        self.ref_status.update({
            'is_error': False,
            'error_message': "",
            'state': 2 # Standby/Working
        })
        self.get_logger().info("✅ Recovery Sequence Completed")

    def call_control_service(self, mode):
        req = SetRobotControl.Request()
        req.robot_control = mode
        future = self.cli_control.call_async(req)
        # 비동기 호출이지만 간단히 로깅만 (결과 대기 X)
        
    def execute_move_line(self, pos):
        if not self.cli_movel.service_is_ready(): return
        req = MoveLine.Request()
        req.pos = pos; req.vel = [100.0, 100.0]; req.acc = [200.0, 200.0]; req.sync_type = 0
        self.cli_movel.call_async(req).add_done_callback(lambda f: self.cb_move_result(f, "MoveLine"))

    def execute_move_joint(self, pos):
        if not self.cli_movej.service_is_ready(): return
        req = MoveJoint.Request()
        req.pos = pos; req.vel = 30.0; req.acc = 30.0; req.sync_type = 0
        self.cli_movej.call_async(req).add_done_callback(lambda f: self.cb_move_result(f, "MoveJoint"))

    def cb_move_result(self, future, move_type):
        try:
            res = future.result()
            if res.success: self.get_logger().info(f"✅ {move_type} Success")
            else: 
                self.get_logger().warn(f"❌ {move_type} Failed")
                self.report_error(f"{move_type} Command Rejected by Controller")
        except Exception as e:
            self.get_logger().error(f"Service Call Error: {e}")

    # ------------------------------------------------------------------
    # Part 3: Monitoring Logic
    # ------------------------------------------------------------------
    def routine_check(self):
        if time.time() - self.last_alive_time > 3.0:
            if self.robot_data['is_online']:
                self.ref_status.update({'is_online': False, 'status': "DISCONNECTED"})
                self.robot_data['is_online'] = False
        
        if self.cli_state.service_is_ready():
            self.cli_state.call_async(GetRobotState.Request()).add_done_callback(self.cb_state)

    def topic_callback_joint(self, msg):
        try:
            self.last_alive_time = time.time()
            joint_map = dict(zip(msg.name, msg.position))
            target_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            joints_deg = [round(math.degrees(joint_map.get(name, 0.0)), 3) for name in target_names]

            if time.time() - self.robot_data['timestamp'] / 1000 > 0.1:
                self.robot_data['joint'] = joints_deg
                self.robot_data['timestamp'] = int(time.time() * 1000)
                self.robot_data['is_online'] = True
                self.ref_status.update({'joint': joints_deg, 'timestamp': self.robot_data['timestamp'], 'is_online': True})
        except: pass

    def cb_state(self, future):
        try:
            res = future.result()
            curr_code = res.robot_state
            self.last_alive_time = time.time()
            self.robot_data['state_code'] = curr_code
            status_text = ROBOT_STATE_MAP.get(curr_code, f"UNKNOWN({curr_code})")
            self.robot_data['status'] = status_text
            
            # [에러 자동 감지 및 보고]
            if curr_code in ERROR_STATE_CODES:
                err_msg = f"Robot Error: {status_text}"
                # 중복 보고 방지 (이미 에러상태면 패스)
                # self.report_error(err_msg) # 너무 빈번할 수 있으므로 필요시 주석 해제
                
                # 대신 현재 상태만 업데이트
                self.ref_status.update({'state': 3, 'is_error': True, 'error_message': err_msg})

            if self.cli_mode.service_is_ready():
                self.cli_mode.call_async(GetRobotMode.Request()).add_done_callback(self.cb_mode)
        except: pass

    def cb_mode(self, future):
        try:
            res = future.result()
            self.robot_data['mode'] = res.robot_mode
            if self.cli_tcp.service_is_ready():
                self.cli_tcp.call_async(GetCurrentPosx.Request()).add_done_callback(self.cb_tcp)
        except: pass

    def cb_tcp(self, future):
        try:
            res = future.result()
            if hasattr(res, 'task_pos_info'): raw = res.task_pos_info[0].data
            elif hasattr(res, 'posx'): raw = res.posx
            else: return

            self.robot_data['tcp'] = {
                'x': round(raw[0], 2), 'y': round(raw[1], 2), 'z': round(raw[2], 2),
                'rx': round(raw[3], 2), 'ry': round(raw[4], 2), 'rz': round(raw[5], 2)
            }
            
            # 에러 상태가 아닐 때만 상태 코드 업데이트 (에러 덮어쓰기 방지)
            state_val = 3 if self.robot_data['state_code'] in ERROR_STATE_CODES else 2
            
            self.ref_status.update({
                'tcp': self.robot_data['tcp'],
                'mode': self.robot_data['mode'],
                'status': self.robot_data['status'],
                'state': state_val
            })
        except: pass

    def report_error(self, message):
        self.ref_status.update({'is_error': True, 'error_message': message, 'state': 3})

def main(args=None):
    rclpy.init(args=args)
    node = BeauroFullSystem()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ref_status.update({'is_online': False, 'status': "DISCONNECTED"})
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
