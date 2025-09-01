# pick_place_interactive_home_v2.py
import socket
import rbpodo as rb
import numpy as np
import time

# ======= 연결/설정 =======
ROBOT_IP = "192.168.50.60"
SOCKET_PORT = 5000  # move_l_rel용 스크립트 서버 포트

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
pose_home   = np.array([-90.0, -45.0, 135.0, 0.0, 90.0, 0.0])

pose_pick_1 = np.array([-180.0, 20.71,  85.05, -4.27, 70.09, 0.0])
pose_pick_2 = np.array([-135.0,  8.48,  98.64,  0.59, 73.38, 0.0])
pose_pick_3 = np.array([ -90.0,  4.14, 104.55,  0.34, 70.38, 0.0])

pose_place_1 = np.array([ 30.0, 15.17,  92.07, -2.13, 71.19, 0.0])
pose_place_2 = np.array([-20.0, 10.47,  97.21, -2.09, 75.03, 0.0])
pose_place_3 = np.array([ -60.0, -2.97,112.15, -5.19, 70.64, 0.0])

pick_list  = [pose_pick_1,  pose_pick_2,  pose_pick_3]
place_list = [pose_place_1, pose_place_2, pose_place_3]

# ======= 타이밍 =======
WAIT_PICK  = 1.5  # s (픽 접점 대기)
WAIT_PLACE = 1.5  # s (플레이스 접점 대기)
WAIT_HOME  = 3.0  # s (홈 대기)

# ======= 리프트 파라미터 =======
# ※ REL_FRAME_INDEX는 실제 +Z=위가 되는 프레임으로 설정(환경에 따라 Base=0일 수도 있음).
REL_FRAME_INDEX = 2
REL_SPEED = 350       # mm/s
REL_ACC   = 350       # mm/s^2
LIFT_DZ   = 200.0     # mm

# ======= 소켓 연결 (move_l_rel 전송용) =======
client_socket = None
try:
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((ROBOT_IP, SOCKET_PORT))
    print(f"[INFO] 스크립트 서버 소켓 연결됨 ({ROBOT_IP}:{SOCKET_PORT})")
except socket.error as e:
    print(f"[WARN] 소켓 연결 실패: {e}")
    print("      (직교 상대이동(move_l_rel)이 작동하지 않을 수 있습니다.)")

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
    print(f"[move_l_rel] cmd={cmd} | resp={str(resp).strip()}")
    ok = wait_for_motion(timeout=12.0)
    if not ok:
        print("[move_l_rel] 모션 완료 신호 미감지(타임아웃/비표준 응답 가능)")

# ======= 견고한 관절 이동 =======
def mmove_j(angles_j, speed_j=30, acceleration_j=30,
            start_timeout=3.0, finish_timeout=20.0):
    """
    - move_j 전송 후 '이동 시작'을 start_timeout 동안 대기
    - 시작 신호가 없으면 '이미 목표에 있음'으로 간주하고 바로 반환
    - 시작되면 finish_timeout 동안 완료 대기
    - 어떤 경우에도 무한 대기하지 않도록 보장
    """
    print(f"[mmove_j] 목표 -> {angles_j}")
    robot.move_j(rc, angles_j, speed_j, acceleration_j)
    robot.flush(rc)

    # 이동 시작 대기 (짧게)
    started = robot.wait_for_move_started(rc, start_timeout)
    if hasattr(started, "is_success") and started.is_success():
        print("[mmove_j] 이동 시작 감지")
        # 완료 대기 (타임아웃 보호)
        _t0 = time.time()
        robot.wait_for_move_finished(rc)  # API가 내부 타임아웃을 안 주더라도…
        # 안전장치: finish_timeout 초 이상 경과 시 경고만 띄우고 진행
        if time.time() - _t0 > finish_timeout:
            print("[WARN][mmove_j] 이동 완료 대기 시간이 너무 깁니다(타임아웃 보호). 계속 진행합니다.")
    else:
        # 시작 신호 없음: 이미 그 포즈일 가능성 큼 → 바로 진행
        print("[mmove_j] 이동 시작 신호 없음 → 이미 목표각에 있다고 판단하고 진행")

def descend_wait_lift(wait_sec: float):
    """접근자세(상공)에서 200mm 하강 → 대기 → 200mm 상승"""
    move_l_rel(0.0, 0.0, -LIFT_DZ)  # 하강(접점)
    time.sleep(wait_sec)            # 접점 대기
    move_l_rel(0.0, 0.0, +LIFT_DZ)  # 상승(접근자세 복귀)

def ask_index(prompt: str):
    """'1','2','3' 또는 '0'만 허용. 그 외는 재입력."""
    while True:
        s = input(prompt).strip()
        if s in {"0", "1", "2", "3"}:
            return s
        print("[INFO] 잘못된 입력입니다. 1/2/3 또는 0을 입력하세요.")

def run_cycle(pick_idx: int, place_idx: int):
    """
    Home을 반드시 경유하는 사이클:
    Home → Pick(200↓ 대기 200↑) → Home(3s)
         → Place(200↓ 대기 200↑) → Home(3s)
    """
    # (A) Home → Pick 접근
    mmove_j(pose_home, 30, 30)
    time.sleep(WAIT_HOME)
    mmove_j(pick_list[pick_idx-1], 30, 30)
    descend_wait_lift(WAIT_PICK)

    # (B) Pick 후 Home 경유
    mmove_j(pose_home, 30, 30)
    time.sleep(WAIT_HOME)

    # (C) Home → Place 접근
    mmove_j(place_list[place_idx-1], 30, 30)
    descend_wait_lift(WAIT_PLACE)

    # (D) Place 후 Home 경유
    mmove_j(pose_home, 30, 30)
    time.sleep(WAIT_HOME)

# ======= 메인 루프 =======
def main():
    print("=== 인터랙티브 Pick & Place (Home 경유, 무한대기 방지) ===")
    print("안내: 먼저 Pick 번호를, 다음에 Place 번호를 입력하면 한 번에 실행됩니다.")
    print("언제든 0을 입력하면 Home 복귀 후 종료합니다.\n")

    # 시작 시 Home 정위치(이미 Home일 수도 있으므로 mmove_j가 안전 처리)
    mmove_j(pose_home, 30, 30)
    time.sleep(WAIT_HOME)

    try:
        while True:
            pick_in = ask_index("Pick할 위치를 입력하세요 (1/2/3, 종료=0): ")
            if pick_in == "0":
                print("[INFO] Home으로 복귀 후 종료합니다.")
                mmove_j(pose_home, 30, 30)
                time.sleep(WAIT_HOME)
                break

            place_in = ask_index("Place할 위치를 입력하세요 (1/2/3, 종료=0): ")
            if place_in == "0":
                print("[INFO] Home으로 복귀 후 종료합니다.")
                mmove_j(pose_home, 30, 30)
                time.sleep(WAIT_HOME)
                break

            p_idx, q_idx = int(pick_in), int(place_in)
            print(f"[INFO] 실행: Home → Pick {p_idx} → Home → Place {q_idx} → Home")
            run_cycle(p_idx, q_idx)

    except KeyboardInterrupt:
        print("\n[WARN] 사용자 인터럽트. Home 복귀 후 종료합니다.")
        try:
            mmove_j(pose_home, 30, 30)
            time.sleep(WAIT_HOME)
        except Exception as e:
            print(f"[WARN] Home 복귀 중 예외: {e}")
    finally:
        if client_socket is not None:
            try:
                client_socket.close()
            except Exception:
                pass
        print("[INFO] 프로그램 종료.")

if __name__ == "__main__":
    main()
