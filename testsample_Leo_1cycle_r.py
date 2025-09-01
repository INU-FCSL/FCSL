# pick_and_place_lift.py
import socket
import rbpodo as rb
import numpy as np
import time

# ======= 설정 =======
ROBOT_IP = "192.168.50.60"
SOCKET_PORT = 5000  # 스크립트 서버 포트
print(f"[INFO] 로봇에 연결 중... IP: {ROBOT_IP}")

robot = rb.Cobot(ROBOT_IP)
rc = rb.ResponseCollector()
print("[INFO] 로봇 연결 성공.")

# 실동작 모드
robot.set_operation_mode(rc, rb.OperationMode.Real)

# 전체 속도 오버라이드 (70%)
speed_override = 0.4
robot.set_speed_bar(rc, speed_override)

# ======= 좌표(절대 관절각) =======
pose_home   = np.array([-90.0, -45.0, 135.0, 0.0, 90.0, 0.0])

pose_pick_1 = np.array([-137.9, 40.93, 93.73, -6.06, 41.3, 0.0])
pose_pick_2 = np.array([-124.53, 34.26, 103.82, -6.29, 39.76, 0.0])
pose_pick_3 = np.array([-101.92, 27.6, 117.61, 0.61, 29.61, 0.0])

pose_place_1 = np.array([22.06, 15.17, 92.07, -2.13, 71.19, 0.0])
pose_place_2 = np.array([8.27, 10.47, 97.21, -2.09, 75.03, 0.0])
pose_place_3 = np.array([-5.6, -2.97, 112.15, -5.19, 70.64, 0.0])

pick_list  = [pose_pick_1,  pose_pick_2,  pose_pick_3]
place_list = [pose_place_1, pose_place_2, pose_place_3]

# ======= 타이밍 =======
WAIT_PICK  = 1.5  # s (픽 지점 대기)
WAIT_PLACE = 1.5  # s (플레이스 지점 대기)
WAIT_HOME  = 3.0  # s (홈에서 대기)

# ======= 리프트(상하) 파라미터 =======
# 좌표계 인덱스는 환경에 맞게 확인(예: 0=Base, 1=Tool, 2=User0 등)
REL_FRAME_INDEX = 2
REL_SPEED = 350     # mm/s (상대이동 속도)
REL_ACC   = 350     # mm/s^2 (상대이동 가속도)
LIFT_DZ   = 200.0   # mm  ← 요청에 따라 500mm로 상향

# ======= 소켓 연결 (move_l_rel 전송용) =======
client_socket = None
try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ROBOT_IP, SOCKET_PORT))
    print(f"[INFO] 스크립트 서버 소켓 연결됨 ({ROBOT_IP}:{SOCKET_PORT})")
except socket.error as e:
    print(f"[WARN] 소켓 연결 실패: {e}")
    print("      (직교 상대이동이 작동하지 않을 수 있습니다.)")

def send_command(command: str) -> str:
    if client_socket is None:
        return "Error: Socket not connected"
    try:
        client_socket.sendall(command.encode())
        resp = client_socket.recv(1024).decode()
        return resp
    except socket.error as e:
        return f"Error: {e}"

def wait_for_motion(timeout=15.0) -> bool:
    # 리프트 거리가 커져서 타임아웃을 넉넉히 15초로
    if client_socket is None:
        return False
    client_socket.settimeout(timeout)
    start = time.time()
    try:
        while time.time() - start < timeout:
            data = client_socket.recv(1024).decode()
            if "info[motion_changed][0]" in data:
                client_socket.settimeout(None)
                return True
        client_socket.settimeout(None)
        return False
    except (socket.timeout, socket.error):
        client_socket.settimeout(None)
        return False

def move_l_rel(dx, dy, dz, drx=0.0, dry=0.0, drz=0.0,
               v=REL_SPEED, a=REL_ACC, frame=REL_FRAME_INDEX):
    cmd = f"move_l_rel(pnt[{dx}, {dy}, {dz}, {drx}, {dry}, {drz}], {v}, {a}, {frame})"
    resp = send_command(cmd)
    print(f"[move_l_rel] cmd={cmd} | resp={resp.strip()}")
    ok = wait_for_motion(timeout=15.0)
    if not ok:
        print("[move_l_rel] 모션 완료 신호 미감지")

# ======= 관절 절대이동 =======
def mmove_j(angles_j, speed_j=30, acceleration_j=30):
    print(f"[mmove_j] 이동 -> {angles_j}")
    robot.move_j(rc, angles_j, speed_j, acceleration_j)
    robot.flush(rc)
    robot.wait_for_move_started(rc, 5.0)
    robot.wait_for_move_finished(rc)

# ======= 메인 로직 =======
def main():
    print("=== Pick & Place (Lift=500mm / Home 경유) 시작 ===")

    # 시작: Home 이동 및 3초 대기
    mmove_j(pose_home, 30, 30)
    time.sleep(WAIT_HOME)

    for i in range(3):
        print(f"\n--- {i+1}번째 사이클 ---")

        # 1) Pick_i 이동 + 1.5s 대기
        mmove_j(pick_list[i], 30, 30)
        time.sleep(WAIT_PICK)

        # 2) Pick 지점에서 위로 500mm 리프트
        move_l_rel(0.0, 0.0, +LIFT_DZ)

        # 3) Home 경유(상공에서 홈으로 관절 이동)
        mmove_j(pose_home, 30, 30)

        # 4) Place_i로 관절 이동
        mmove_j(place_list[i], 30, 30)

        # 5) Place 지점으로 500mm 내려가기 + 1.5s 대기
        move_l_rel(0.0, 0.0, -LIFT_DZ)
        time.sleep(WAIT_PLACE)

        # 6) 다시 500mm 위로 리프트
        move_l_rel(0.0, 0.0, +LIFT_DZ)

        # 7) Home 복귀 + 3s 대기
        mmove_j(pose_home, 30, 30)
        time.sleep(WAIT_HOME)

    print("\n=== 모든 Pick & Place 완료 ===")

if __name__ == "__main__":
    main()
