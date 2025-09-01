import rbpodo as rb
import numpy as np
import time

# ======= 설정 부분 =======
ROBOT_IP = "192.168.0.22"
print(f"로봇에 연결 중... IP: {ROBOT_IP}")
robot = rb.Cobot(ROBOT_IP)
rc = rb.ResponseCollector()
print("로봇 연결 성공.")

# 실동작 모드
robot.set_operation_mode(rc, rb.OperationMode.Real)

# 속도 오버라이드 (안전을 위해 30%로 시작 권장)
speed_override = 0.3
robot.set_speed_bar(rc, speed_override)

# ======= 좌표 정의 (절대 관절 각도) =======
pose_home   = np.array([-90.0, -45.0, 135.0, 0.0, 90.0, 0.0])

pose_pick_1 = np.array([-137.9, 40.93, 93.73, -6.06, 41.3, 0.0])
pose_pick_2 = np.array([-124.53, 34.26, 103.82, -6.29, 39.76, 0.0])
pose_pick_3 = np.array([-101.92, 27.6, 117.61, 0.61, 29.61, 0.0])

pose_place_1 = np.array([22.06, 36.91, 100.80, -3.06, 40.77, 0.0])
pose_place_2 = np.array([8.27, 33.25, 106.40, -2.92, 43.08, 0.0])
pose_place_3 = np.array([-5.6, 24.8, 121.52, -11.7, 32.91, 0.0])

pick_list  = [pose_pick_1, pose_pick_2, pose_pick_3]
place_list = [pose_place_1, pose_place_2, pose_place_3]

# ======= 이동 함수 =======
def mmove_j(angles_j, speed_j=30, acceleration_j=30):
    print(f"[mmove_j] 이동 -> {angles_j}")
    robot.move_j(rc, angles_j, speed_j, acceleration_j)
    robot.flush(rc)
    robot.wait_for_move_started(rc, 5.0)
    robot.wait_for_move_finished(rc)

# ======= 메인 로직 =======
def main():
    print("=== Pick and Place 시작 ===")

    # 시작 시 Home으로 이동
    mmove_j(pose_home, 30, 30)
    time.sleep(1.0)

    for i in range(3):  # Pick1→Place1, Pick2→Place2, Pick3→Place3
        print(f"\n--- {i+1}번째 사이클 ---")

        # Pick 이동 + 0.5초 대기
        mmove_j(pick_list[i], 30, 30)
        time.sleep(0.5)

        # Place 이동 + 0.5초 대기
        mmove_j(place_list[i], 30, 30)
        time.sleep(0.5)

        # 다시 Home 이동 + 1초 대기
        mmove_j(pose_home, 30, 30)
        time.sleep(1.0)

    print("\n=== 모든 Pick and Place 완료 ===")

if __name__ == "__main__":
    main()
