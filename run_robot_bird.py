import os
import sys
import time
import math
import msvcrt
import mujoco
import mujoco.viewer
import numpy as np

# UTF-8 입출력 보장
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

from robot_bird.controller import RobotBirdFSM
from robot_bird.recorder import ScreenRecorder

def print_banner():
    print("=" * 80)
    print("      🎮 Control Guide (반려 로봇새 조종 키 안내)")
    print("=" * 80)
    print(" 🌟 [ 선회 회전 (Yaw Turn) ] : A / D       ➔ 머리 왼쪽 / 오른쪽 회전 (비행 뱅크턴 / 보행 회전)")
    print(" 🌟 [ 전진 / 후진          ] : ↑ / ↓ (W/S) ➔ 지상: 전진/후진 걷기 | 공중: 바라보는 방향 전진/후진 비행")
    print(" 🌟 [ 평행 이동 (Strafe)   ] : ← / →       ➔ 회전 없이 좌 / 우 평행 슬라이드 이동")
    print(" 🌟 [ 고도 제어            ] : R / F       ➔ 🚀 고도 상승 (+15cm) | 🛬 고도 하강 (-15cm)")
    print(" 🌟 [ 제자리 정지          ] : SPACE       ➔ 🛑 지상: 기립 정지 | 공중: 칼 같은 정지 호버링")
    print(" 🌟 [ 원터치 비행          ] : 1 ➔ 2 ➔ 3   ➔ 🪽 날개 펼치기 ➔ 🚀 이륙/호버링 ➔ 🛬 착륙 및 날개 접기")
    print(" 🌟 [ 📷 눈 카메라 시점    ] : C           ➔ 👁️ 3인칭 뷰 ➔ 중앙 1인칭 FPV ➔ 좌측 눈 ➔ 우측 눈")
    print(" 🌟 [ 👀 눈알 시선 조작    ] : V 또는 B     ➔ 👇 눈알 아래 55° 보기(V) 🔁 다시 눈알 원래 위치로 복귀(V/B)")
    print(" 🌟 [ 🔴 실시간 영상 녹화  ] : X           ➔ 🎬 현재 화면 실시간 표준 H.264 MP4 동영상 녹화 ON / OFF")
    print(" 🌟 [ 감성 모션 & 인터랙션 ] : G / Y / K / P ➔ 🌾 모이 쪼기(G) | 🪑 쪼그려 앉기(Y) | 💥 넘어뜨리기(K) | 🔄 오뚝이 기립(P)")
    print(" 🌟 [ 시뮬레이터 종료      ] : Q / ESC")
    print("=" * 80)
    print("\n시뮬레이터를 시작합니다... (3D 창을 클릭하고 멋지게 조종해보세요!)\n")

def handle_key_action(keycode, controller, recorder=None):
    """
    Control Guide 표와 100% 일치하는 직관적인 키 핸들러
    """
    is_flying = controller.state in [
        RobotBirdFSM.STATE_FLIGHT,
        RobotBirdFSM.STATE_TAKEOFF_CLIMB,
        RobotBirdFSM.STATE_TAKEOFF_SPOOL,
        RobotBirdFSM.STATE_LANDING
    ]

    # [ Q / ESC ] : 시뮬레이터 종료
    if keycode in [ord('Q'), ord('q'), 256, 27]:
        print("\n시뮬레이터를 종료합니다. 안녕히 가세요!")
        sys.exit(0)

    # [ 1 ] : 날개 펼치기 / 접기
    elif keycode in [ord('1'), 49, 321, ord('E'), ord('e')]:
        controller.toggle_wings()

    # [ 2 ] : 이륙 및 1.0m 정지 호버링
    elif keycode in [ord('2'), 50, 322, 257, ord('T'), ord('t')]:
        controller.start_flight_sequence()

    # [ 3 ] : 착륙 및 날개 접기
    elif keycode in [ord('3'), 51, 323, ord('H'), ord('h'), 259]:
        controller.set_state(RobotBirdFSM.STATE_LANDING)

    # [ ↑ ] / [ W ] : 전진 (지상: 전진 걷기 / 공중: 전방 비행)
    elif keycode in [265, 38, ord('W'), ord('w')]:
        if is_flying:
            yaw = getattr(controller, 'target_yaw', 0.0)
            controller.target_x += 0.30 * (-math.sin(yaw))
            controller.target_y += 0.30 * math.cos(yaw)
            print(f"\n[🚀 전진 비행] 바라보는 방향으로 30cm 전진! (X:{controller.target_x:.2f}m, Y:{controller.target_y:.2f}m)")
        else:
            print(f"\n[🚶 보행 모드] ⬆️ 앞으로 아장아장 걷기 시작!")
            controller.walk_speed = 1.0
            controller.walk_turn = 0.0
            controller.set_state(RobotBirdFSM.STATE_WALK)

    # [ ↓ ] / [ S ] : 후진 (지상: 후진 걷기 / 공중: 후방 비행)
    elif keycode in [264, 40, ord('S'), ord('s')]:
        if is_flying:
            yaw = getattr(controller, 'target_yaw', 0.0)
            controller.target_x -= 0.30 * (-math.sin(yaw))
            controller.target_y -= 0.30 * math.cos(yaw)
            print(f"\n[🛬 후진 비행] 바라보는 반대로 30cm 후진! (X:{controller.target_x:.2f}m, Y:{controller.target_y:.2f}m)")
        else:
            print(f"\n[🚶 보행 모드] ⬇️ 뒤로 아장아장 걷기 시작!")
            controller.walk_speed = -0.7
            controller.walk_turn = 0.0
            controller.set_state(RobotBirdFSM.STATE_WALK)

    # [ ← ] : 좌측 평행 이동 (공중: 좌 슬라이드 / 지상: 좌회전 걷기)
    elif keycode in [263, 37]:
        if is_flying:
            yaw = getattr(controller, 'target_yaw', 0.0)
            controller.target_x += 0.30 * math.cos(yaw)
            controller.target_y += 0.30 * math.sin(yaw)
            print(f"\n[⬅️ 평행 슬라이드] 왼쪽으로 30cm 이동! (X:{controller.target_x:.2f}m, Y:{controller.target_y:.2f}m)")
        else:
            print(f"\n[🚶 보행 모드] ⬅️ 왼쪽으로 방향을 틀며 걷기")
            controller.walk_speed = 0.5
            controller.walk_turn = 0.7
            controller.set_state(RobotBirdFSM.STATE_WALK)

    # [ → ] : 우측 평행 이동 (공중: 우 슬라이드 / 지상: 우회전 걷기)
    elif keycode in [262, 39]:
        if is_flying:
            yaw = getattr(controller, 'target_yaw', 0.0)
            controller.target_x -= 0.30 * math.cos(yaw)
            controller.target_y -= 0.30 * math.sin(yaw)
            print(f"\n[➡️ 평행 슬라이드] 오른쪽으로 30cm 이동! (X:{controller.target_x:.2f}m, Y:{controller.target_y:.2f}m)")
        else:
            print(f"\n[🚶 보행 모드] ➡️ 오른쪽으로 방향을 틀며 걷기")
            controller.walk_speed = 0.5
            controller.walk_turn = -0.7
            controller.set_state(RobotBirdFSM.STATE_WALK)

    # [ A ] : 머리 왼쪽 회전 (공중: 좌선회 턴 / 지상: 좌회전 걷기)
    elif keycode in [ord('A'), ord('a')]:
        if is_flying:
            controller.target_yaw = getattr(controller, 'target_yaw', 0.0) - 0.25
            deg = math.degrees(controller.target_yaw) % 360
            print(f"\n[🔄 선회 회전] 머리 왼쪽으로 회전! (목표 각도: {deg:.1f}°)")
        else:
            print(f"\n[🚶 보행 모드] 🔄 왼쪽으로 방향을 틀며 걷기")
            controller.walk_speed = 0.5
            controller.walk_turn = 0.7
            controller.set_state(RobotBirdFSM.STATE_WALK)

    # [ D ] : 머리 오른쪽 회전 (공중: 우선회 턴 / 지상: 우회전 걷기)
    elif keycode in [ord('D'), ord('d')]:
        if is_flying:
            controller.target_yaw = getattr(controller, 'target_yaw', 0.0) + 0.25
            deg = math.degrees(controller.target_yaw) % 360
            print(f"\n[🔄 선회 회전] 머리 오른쪽으로 회전! (목표 각도: {deg:.1f}°)")
        else:
            print(f"\n[🚶 보행 모드] 🔄 오른쪽으로 방향을 틀며 걷기")
            controller.walk_speed = 0.5
            controller.walk_turn = -0.7
            controller.set_state(RobotBirdFSM.STATE_WALK)

    # [ R ] / [ + ] : 고도 상승 (+15cm)
    elif keycode in [ord('R'), ord('r'), ord('+'), ord('='), 266, 33, 334]:
        if is_flying:
            controller.target_altitude += 0.15
            print(f"\n[🚀 고도 상승] 위로 상승! (목표 고도: {controller.target_altitude:.2f}m)")

    # [ F ] / [ - ] : 고도 하강 (-15cm)
    elif keycode in [ord('F'), ord('f'), ord('-'), ord('_'), 267, 34, 333]:
        if is_flying:
            controller.target_altitude = max(0.20, controller.target_altitude - 0.15)
            print(f"\n[🛬 고도 하강] 아래로 하강! (목표 고도: {controller.target_altitude:.2f}m)")

    # [ SPACE ] : 제자리 정지 (지상: 기립 정지 / 공중: 칼 같은 정지 호버링)
    elif keycode in [ord(' '), 32]:
        if is_flying:
            print(f"\n[🛑 정지 호버링] 현재 위치에서 칼 같이 정지 호버링합니다.")
            controller.target_x = controller.data.qpos[0]
            controller.target_y = controller.data.qpos[1]
        else:
            print(f"\n[🦆 기립 정지] 걷기를 멈추고 반듯하게 제자리에 섭니다.")
            controller.walk_speed = 0.0
            controller.walk_turn = 0.0
            controller.set_state(RobotBirdFSM.STATE_STAND)

    # [ C ] : 눈에서 직접 보는 시점 전환 (3인칭 관찰 ➔ 1인칭 얼굴 중심 ➔ 좌측 눈 직접 뷰 ➔ 우측 눈 직접 뷰)
    elif keycode in [ord('C'), ord('c')]:
        if not hasattr(controller, 'cam_mode'):
            controller.cam_mode = 0
        controller.cam_mode = (controller.cam_mode + 1) % 4
        cam_names = [
            "🕊️ 3인칭 전체 뷰 (외부에서 로봇새 모습 관찰)",
            "📷 [1인칭 FPV] 로봇새 머리 정중앙 시점",
            "👁️ [좌측 눈 직접 시점] 로봇새의 왼쪽 눈알에서 세상을 직접 바라보는 시점!",
            "👁️ [우측 눈 직접 시점] 로봇새의 오른쪽 눈알에서 세상을 직접 바라보는 시점!"
        ]
        print(f"\n[📷 시점 전환] 현재 시점: {cam_names[controller.cam_mode]}")

    # [ V ] : 머리/눈알 하방 시선 토글 (아래 55° 보기 <-> 정면 똑바로 보기)
    elif keycode in [ord('V'), ord('v')]:
        controller.toggle_look_down()

    # [ B ] : 머리/눈알 즉시 정면 수평 시선으로 복귀
    elif keycode in [ord('B'), ord('b')]:
        controller.look_front()

    # [ X ] : 실시간 표준 H.264 MP4 비디오 녹화 ON / OFF 토글
    elif keycode in [ord('X'), ord('x')]:
        if recorder is not None:
            cam_mode = getattr(controller, 'cam_mode', 0)
            recorder.toggle(cam_mode)

    # [ G ] : 다리 웅크림 3단 모이 쪼기
    elif keycode in [ord('G'), ord('g')]:
        print(f"\n[🌾 감성 모션] 다리를 쏙 접으며 바닥에 콕! 콕! 콕! 3단 모이 쪼기!")
        controller.set_state(RobotBirdFSM.STATE_GROUND_PICK)

    # [ Y ] : 쪼그려 앉기 / 일어서기 토글
    elif keycode in [ord('Y'), ord('y')]:
        if controller.state == RobotBirdFSM.STATE_SIT:
            print(f"\n[🪑 감성 모션] 쪼그려 앉기에서 다시 일어섭니다.")
            controller.set_state(RobotBirdFSM.STATE_STAND)
        else:
            print(f"\n[🪑 감성 모션] 다리를 쏙 접고 얌전히 쪼그려 앉습니다.")
            controller.set_state(RobotBirdFSM.STATE_SIT)

    # [ K ] : 꽈당 넘어뜨리기 테스트
    elif keycode in [ord('K'), ord('k'), ord('O'), ord('o')]:
        print(f"\n[💥 넘어뜨리기] 꽈당! 로봇이 누웠습니다. ('P' 키를 누르면 벌떡 일어납니다!)")
        controller.data.qvel[0] = 0.35
        controller.data.qvel[3] = 4.5
        controller.data.qvel[4] = 2.0
        controller.set_state(RobotBirdFSM.STATE_KNOCKDOWN)

    # [ P ] : 3D 오뚝이 벌떡 기립 (누워있을 때: 기립 / 서 있을 때: 재롱 점프 댄스)
    elif keycode in [ord('P'), ord('p')]:
        print(f"\n[🔄 오뚝이 기립] 벌떡 일어서기 & 갸우뚱 재롱 댄스!")
        controller.set_state(RobotBirdFSM.STATE_RECOVER)

def main():
    print_banner()

    root_dir = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(root_dir, "robot_bird", "models", "robot_bird.xml")

    if not os.path.exists(model_path):
        print(f"[오류] 모델 파일을 찾을 수 없습니다: {model_path}")
        return

    with open(model_path, "r", encoding="utf-8") as f:
        xml_string = f.read()
    model = mujoco.MjModel.from_xml_string(xml_string)
    data = mujoco.MjData(model)

    # 🌟 관절 이름 기반의 안전하고 정확한 초기 자세 설정 (보행 및 날개 자세 100% 보장)
    def set_init_jnt(jnt_name, val):
        adr = model.joint(jnt_name).qposadr[0]
        data.qpos[adr] = val

    data.qpos[2] = 0.122 # z 높이 (지면 완벽 안착)
    set_init_jnt("left_wing_fold", 0.0)
    set_init_jnt("left_tilt", 1.57)
    set_init_jnt("right_wing_fold", 0.0)
    set_init_jnt("right_tilt", 1.57)
    set_init_jnt("neck_pitch", 0.0)
    for j in ["left_hip_pitch", "left_knee_pitch", "left_ankle_pitch", 
              "right_hip_pitch", "right_knee_pitch", "right_ankle_pitch"]:
        set_init_jnt(j, 0.0)

    data.ctrl[:] = 0.0
    data.ctrl[model.actuator("act_l_tilt").id] = 1.57
    data.ctrl[model.actuator("act_r_tilt").id] = 1.57
    mujoco.mj_forward(model, data)

    controller = RobotBirdFSM(model, data)
    recorder = ScreenRecorder(model, data)

    def viewer_key_callback(keycode):
        handle_key_action(keycode, controller, recorder)

    with mujoco.viewer.launch_passive(model, data, key_callback=viewer_key_callback, show_left_ui=False, show_right_ui=False) as viewer:
        try:
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = 0
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 0
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONSTRAINT] = 0
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = 0
        except Exception:
            pass

        while viewer.is_running():
            step_start = time.time()

            if msvcrt.kbhit():
                key = msvcrt.getch()
                if key in [b'\x00', b'\xe0']:
                    sub = msvcrt.getch()
                    if sub == b'H': handle_key_action(265, controller, recorder)      # UP Arrow
                    elif sub == b'P': handle_key_action(264, controller, recorder)    # DOWN Arrow
                    elif sub == b'K': handle_key_action(263, controller, recorder)    # LEFT Arrow
                    elif sub == b'M': handle_key_action(262, controller, recorder)    # RIGHT Arrow
                else:
                    try:
                        k_ord = ord(key.decode('utf-8', errors='ignore'))
                        handle_key_action(k_ord, controller, recorder)
                        if k_ord in [ord('q'), ord('Q'), 27]:
                            break
                    except Exception:
                        pass

            controller.cmd_pitch *= 0.95
            controller.cmd_roll *= 0.95
            controller.cmd_yaw *= 0.95

            dt = model.opt.timestep
            controller.update(dt)
            mujoco.mj_step(model, data)

            # 🌟 [힘 표시선/번개선 차단 & 바닥 텍스처/그리드 100% 선명하게 보존]
            try:
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = 0
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = 0
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONSTRAINT] = 0
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = 0
            except Exception:
                pass

            # 🌟 [스마트 카메라 시점 제어] (C 키로 3인칭 <-> 1인칭 FPV / 눈 카메라 전환)
            cam_mode = getattr(controller, 'cam_mode', 0)
            try:
                if cam_mode == 0:
                    # 3인칭 스마트 추적 뷰
                    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                    viewer.cam.lookat[0] = data.qpos[0]
                    viewer.cam.lookat[1] = data.qpos[1]
                    viewer.cam.lookat[2] = data.qpos[2]
                elif cam_mode == 1:
                    # 📷 중앙 FPV 카메라
                    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                    viewer.cam.fixedcamid = model.camera("head_fpv_cam").id
                elif cam_mode == 2:
                    # 👁️ 좌측 눈 카메라
                    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                    viewer.cam.fixedcamid = model.camera("left_eye_cam").id
                elif cam_mode == 3:
                    # 👁️ 우측 눈 카메라
                    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                    viewer.cam.fixedcamid = model.camera("right_eye_cam").id
            except Exception:
                pass

            # 🔴 실시간 비디오 프레임 캡처 (녹화 중일 때 자동 기록)
            if recorder.is_recording:
                recorder.capture_frame(cam_mode)

            viewer.sync()

            elapsed = time.time() - step_start
            sleep_time = max(0.0, 0.004 - elapsed)
            if sleep_time > 0:
                time.sleep(sleep_time)

        # 프로그램 종료 시 녹화 자동 마무리 저장
        if recorder.is_recording:
            recorder.stop()

if __name__ == "__main__":
    main()
