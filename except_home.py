# pick_and_place_direct.py
import socket
import rbpodo as rb
import numpy as np
import time

# ======= 연결/설정 =======
ROBOT_IP = "192.168.50.60"
SOCKET_PORT = 5000  # 로봇 스크립트 서버 포트 (move_l_rel용)

print(f"[INFO] 로봇에 연결 중... IP: {ROBOT_IP}")
robot = rb.Cobot(ROBOT_IP)
rc = rb.ResponseCollector()
print("[INFO] 로봇 연결 성공.")

# 실동작 모드
robot.set_operation_mode(rc, rb.OperationMode.Real)

# 전체 속도 오버라이드(70%)
speed_override = 0.7
robot.set_speed_bar(rc, speed_override)

# ======= 관절 포즈 정의 (모두 상공 +200mm 접근자세) =======
pose_pick_1 = np.array([-180.0, 20.71, 85.05, -4.27, 70.09, 0.0])
pose_pick_2 = np.array([-135.0, 8.48, 98.64,  0.59, 73.38, 0.0])
pose_pick_3 = np.array([-90.0,  4.14,104.55,  0.34, 70.38, 0.0])

pose_place_1 = np.array([30.0, 15.17, 92.07, -2.13, 71.19, 0.0])
pose_place_2 = np.array([-20.0, 10.47, 97.21, -2.09, 75.03, 0.0])
pose_place_3 = np.array([-60.0, -2.97, 112.15, -5.19, 70.64, 0.0])

pick_list  = [pose_pick_1,  pose_pick_2,  pose_pick_3]
place_list = [pose_place_1, pose_place_2, pose_place_3]

# ======= 타이밍 =======
WAIT_PICK  = 1.5  # s (픽 접점에서 대기)
WAIT_PLACE = 1.5  # s (플레이스 접점에서 대기)

# ======= 수직 리프트(상하) 파라미터 =======
# ※ REL_FRAME_INDEX는 반드시 실제로 +Z가 '위'인 프레임으로 맞춰야 합니다.
REL_FRAME_INDEX = 2   # 예: 0=Base, 1=Tool, 2=User0 ... (환경에 맞게 확인)
REL_SPEED = 350       # mm/s  (상대직교이동 속도)
REL_ACC   = 350       # mm/s^2
LIFT_DZ   = 200.0     # mm    (요청: 접근자세에서 200mm 내려갔다가 다시 200mm 상승)

# ======= 소켓 연결 (move_l_rel 전송용) =======
client_socket = None
try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ROBOT_IP, SOCKET_PORT))
    print(f"[INFO] 스크립트 서버 소켓 연결됨 ({ROBOT_IP}:{SOCKET_PORT})")
except socket.error as e:
    print(f"[WARN] 소켓 연결 실패: {e}")
    print("      (직교 상대이동이 작동하지 않을 수 있습니다. move_l_rel 사용불가)")

def send_command(command: str) -> str:
    if client_socket is None:
        return "Error: Socket not connected"
    try:
        client_socket.sendall(command.encode())
        resp = client_socket.recv(1024).decode()
        return resp
    except socket.error as e:
        return f"Error: {e}"

def wait_for_motion(timeout=12.0) -> bool:
    # 간단한 완료 감지(장비 응답 포맷에 따라 조정 필요)
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
    """
    현재 TCP 기준 상대 직선 이동 (직교 보간):
    move_l_rel(pnt[dx,dy,dz,drx,dry,drz], v, a, frame)
    """
    cmd = f"move_l_rel(pnt[{dx}, {dy}, {dz}, {drx}, {dry}, {drz}], {v}, {a}, {frame})"
    resp = send_command(cmd)
    print(f"[move_l_rel] cmd={cmd} | resp={str(resp).strip()}")
    ok = wait_for_motion(timeout=12.0)
    if not ok:
        print("[move_l_rel] 모션 완료 신호 미감지(타임아웃/비표준 응답 가능)")

def mmove_j(angles_j, speed_j=30, acceleration_j=30):
    """
    관절 공간 절대 이동(PTP). 접근자세 간 장거리 이송은 move_j로,
    접점 근처는 수직 move_l_rel로만 접근/이탈.
    """
    print(f"[mmove_j] 이동 -> {angles_j}")
    robot.move_j(rc, angles_j, speed_j, acceleration_j)
    robot.flush(rc)
    robot.wait_for_move_started(rc, 5.0)
    robot.wait_for_move_finished(rc)

def descend_wait_lift(wait_sec: float):
    """
    접근자세(상공)에서 200mm 하강 → 대기 → 200mm 상승.
    """
    move_l_rel(0.0, 0.0, -LIFT_DZ)  # 하강(접점 도달)
    time.sleep(wait_sec)            # 접점에서 대기
    move_l_rel(0.0, 0.0, +LIFT_DZ)  # 상승(접근자세 복귀)

# ======= 메인 로직 =======
def main():
    print("=== Pick&Place (Home 미경유 / 각 접근자세에서 ±200mm) 시작 ===")

    # 시작은 첫 Pick 접근자세로 바로 이동
    mmove_j(pick_list[0], 30, 30)

    for i in range(3):
        print(f"\n--- {i+1}번째 사이클 ---")

        # (A) Pick_i 접근자세에서 200mm 하강 → 대기 → 상승
        if i > 0:
            # i>0이면 이전 Place에서 바로 pick_i로 넘어온 상태일 수 있으므로
            mmove_j(pick_list[i], 30, 30)
        descend_wait_lift(WAIT_PICK)

        # (B) 곧바로 Place_i 접근자세로 관절이동
        mmove_j(place_list[i], 30, 30)

        # (C) Place_i 접근자세에서 200mm 하강 → 대기 → 상승
        descend_wait_lift(WAIT_PLACE)

        # (D) 다음 사이클로 바로 넘어감(홈 미경유)

    print("\n=== 모든 Pick&Place 완료 ===")

if __name__ == "__main__":
    main()
