import os
import sys
import time
import math
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
from robot_bird.ai_brain import RobotBirdAIBrain
from robot_bird.recorder import ScreenRecorder

class ShowcaseDirector:
    """
    🦅 대표님의 연출 스토리보드를 영화처럼 완벽하게 지휘하는 시네마틱 디렉터
    
    [시나리오 순서 - 리얼 1인칭 눈 시선 비행 & 바둑판 원점 복귀]
    1. 🚶 앞으로 당당하게 걷기 (Forward Walk)
    2. 🚶 조심조심 뒤로 걷기 (Backward Walk)
    3. 💥 꽈당! 앞으로 넘어지기 (Knockdown)
    4. 🔄 오뚝이 지능 발동! 스스로 벌떡 기립 (Self-Righting)
    5. 🌾 바닥 모이 콕! 콕! 쪼아먹기 (Pecking)
    6. 🦿 다리를 쏙 접고 쪼그려 앉았다가 일어서기 (Crouch & Stand)
    
    [🪽 극강의 리얼 시네마틱 비행 & 1인칭 눈 시선]
    7. 🪽 날개 180° 수평 전개 + 3엽 프로펠러 가속 ➔ 1.0m 수직 이륙!
    8. 🚀 [3인칭 전체 뷰] 날개를 펴고 앞으로 출발하는 멋진 전체 모습
    9. 👁️ [1인칭 직접 눈 시점 1 - 전방 비행] 로봇새 눈으로 직접 앞을 바라보며 바람을 가르고 슝~ 날아가는 리얼한 비행 화면!
    10. 👇 [1인칭 직접 눈 시점 2 - 하방 55° 관측] 날아가면서 머리를 55° 아래로 푹 숙여 바닥 체커보드를 직접 내려다보는 리얼한 스캔 화면!
    11. 🔄 [3인칭 전체 뷰] 공중에서 180° 유턴하여 원래 출발했던 바둑판 정중앙(원점)으로 돌아오는 전체 모습!
    12. 🕊️ [3인칭 정지 호버링] 원래 출발했던 바로 그 자리 상공에서 가만히 날고 있는 모습 (Hovering)
    13. 🛬 [사뿐 하강 착지] 원래 출발했던 그 자리에 정확히 수직 착지 & 날개 등 뒤로 접기!
    ➔ (이후 무한 반복!)
    """
    def __init__(self, controller, model, data):
        self.c = controller
        self.m = model
        self.d = data
        self.timeline = 0.0
        self.step_idx = -1
        self.home_x = 0.0
        self.home_y = 0.0

    def update(self, dt):
        self.timeline += dt
        t = self.timeline

        # 1단계: 앞으로 걷기 (0.0s ~ 4.0s)
        if t < 4.0:
            if self.step_idx != 1:
                self.step_idx = 1
                self.home_x = self.d.qpos[0]
                self.home_y = self.d.qpos[1]
                self.c.cam_mode = 0 # 3인칭 전체 뷰
                self.c.look_front()
                self.c.wings_deployed = False
                self.c.set_state(self.c.STATE_WALK)
                print("\n[🎬 1단계] 🚶 [이족보행] 다리를 성큼성큼 번갈아 딛으며 앞으로 걷기")
            self.c.walk_speed = 0.8
            self.c.walk_turn = 0.0

        # 2단계: 뒤로 걷기 (4.0s ~ 7.0s)
        elif t < 7.0:
            if self.step_idx != 2:
                self.step_idx = 2
                self.c.set_state(self.c.STATE_WALK)
                print("\n[🎬 2단계] 🚶 [이족보행] 조심조심 뒤로 걷기 (원점 근처 유지)")
            self.c.walk_speed = -0.8
            self.c.walk_turn = 0.0

        # 3단계: 넘어지기 (7.0s ~ 8.5s)
        elif t < 8.5:
            if self.step_idx != 3:
                self.step_idx = 3
                self.c.walk_speed = 0.0
                self.d.qvel[0] = 0.35
                self.d.qvel[3] = 4.5
                self.d.qvel[4] = 2.0
                self.c.set_state(self.c.STATE_KNOCKDOWN)
                print("\n[🎬 3단계] 💥 꽈당! 균형을 잃고 앞으로 넘어지기 (Knockdown)")

        # 4단계: 오뚝이처럼 다시 일어서기 (8.5s ~ 11.0s)
        elif t < 11.0:
            if self.step_idx != 4:
                self.step_idx = 4
                self.c.set_state(self.c.STATE_RECOVER)
                print("\n[🎬 4단계] 🔄 오뚝이 지능 발동! 스스로 벌떡 일어서기 (Self-Righting)")

        # 5단계: 모이 쪼기 (11.0s ~ 14.5s)
        elif t < 14.5:
            if self.step_idx != 5:
                self.step_idx = 5
                self.c.set_state(self.c.STATE_GROUND_PICK)
                print("\n[🎬 5단계] 🌾 바닥 모이 콕! 콕! 쪼아먹기 (Pecking)")

        # 6단계: 앉았다 일어서기 (14.5s ~ 18.5s)
        elif t < 18.5:
            if self.step_idx != 6:
                self.step_idx = 6
                self.c.set_state(self.c.STATE_SIT)
                print("\n[🎬 6단계] 🦿 다리를 쏙 접고 쪼그려 앉았다가 일어서기 (Crouch & Stand)")
            if t > 16.8 and self.c.state == self.c.STATE_SIT:
                self.c.set_state(self.c.STATE_STAND)

        # 7단계: 날개 180° 전개 & 공중 VTOL 수직 이륙 (18.5s ~ 22.5s)
        elif t < 22.5:
            if self.step_idx != 7:
                self.step_idx = 7
                self.home_x = self.d.qpos[0]
                self.home_y = self.d.qpos[1]
                self.c.target_yaw = 0.0
                self.c.start_flight_sequence()
                print("\n[🎬 7단계] 🪽 등 뒤 날개 180° 전개 + 프로펠러 가속 ➔ 1.0m 수직 이륙!")

        # 8단계: 🚀 [3인칭 뷰] 앞으로 힘차게 출발하는 전체 모습 (22.5s ~ 25.5s)
        elif t < 25.5:
            if self.step_idx != 8:
                self.step_idx = 8
                self.c.look_front()
                self.c.cam_mode = 0 # 3인칭 전체 뷰
                print("\n[🎬 8단계] 🚀 [3인칭 전체 뷰] 날개를 펴고 앞으로 힘차게 출발하는 전체 모습")
            self.c.target_y = self.home_y + 0.45

        # 9단계: 👁️ [1인칭 직접 눈 시점 - 전방 비행] 로봇새 눈으로 직접 앞을 보며 슝~ 날아가는 리얼 속도감! (25.5s ~ 29.5s)
        elif t < 29.5:
            if self.step_idx != 9:
                self.step_idx = 9
                self.c.look_front()
                self.c.cam_mode = 1 # 📷 [1인칭 FPV 직접 눈 시점]
                print("\n" + "=" * 75)
                print(" 👁️ [1인칭 눈 시선 화면 - 전방 비행] 로봇새 눈으로 직접 앞을 바라보며 슝~ 날아갑니다!")
                print("=" * 75)
            self.c.target_y = self.home_y + 0.95

        # 10단계: 🕊️ [3인칭 전체 뷰 - 눈알/머리를 아래로 숙이는 모습] 비행 중 머리를 아래로 푹 숙이는 외형 연출! (29.5s ~ 33.0s)
        elif t < 33.0:
            if self.step_idx != 10:
                self.step_idx = 10
                self.c.cam_mode = 0 # 🌐 [3인칭 전체 뷰로 전환!]
                self.c.target_look_down_pitch = 1.05 # 머리/눈알 아래 60° 깊게 숙이기
                self.c.look_down = True
                print("\n" + "=" * 75)
                print(" 🕊️ [3인칭 전체 뷰] 비행 중인 로봇새가 머리(두 눈)를 아래로 푹 숙여 바닥을 내려다봅니다!")
                print("=" * 75)
            self.c.target_y = self.home_y + 1.15

        # 11단계: 👇 [1인칭 직접 눈 시점 - 하방 60° 관측 화면] 숙인 눈으로 바닥 체커보드를 직접 내려다보는 리얼 시야! (33.0s ~ 37.5s)
        elif t < 37.5:
            if self.step_idx != 11:
                self.step_idx = 11
                self.c.target_look_down_pitch = 1.05
                self.c.look_down = True
                self.c.cam_mode = 1 # 📷 [1인칭 FPV 하방 시점 즉시 컷 전환!]
                print("\n" + "=" * 75)
                print(" 👇 [1인칭 눈 시선 화면 - 하방 60° 관측] 로봇새의 눈으로 직접 바닥 체커보드를 샅샅이 스캔합니다!")
                print("=" * 75)
            self.c.target_y = self.home_y + 1.25

        # 12단계: 🔄 [1인칭 직접 눈 시점 - 180° 선회 유턴] 눈으로 세상을 둘러보며 원점 방향으로 유턴 (37.5s ~ 41.5s)
        elif t < 41.5:
            if self.step_idx != 12:
                self.step_idx = 12
                self.c.look_front() # 고개 정면 복귀
                self.c.cam_mode = 1 # 1인칭 시점 유지하며 유턴
                print("\n[🎬 12단계] 🔄 [1인칭 눈 시선 - 180° 유턴] 고개를 들고 180° 선회하여 출발했던 원점을 눈으로 포착!")
            self.c.target_yaw = math.pi * min(1.0, (t - 37.5) / 3.5)

        # 13단계: 🌐 [3인칭 전체 뷰 복귀] 180도 회전하여 원래 자리(원점)로 귀환하는 전체 모습 (41.5s ~ 46.0s)
        elif t < 46.0:
            if self.step_idx != 13:
                self.step_idx = 13
                self.c.look_front()
                self.c.cam_mode = 0 # 3인칭 전체 뷰 복귀
                self.c.target_yaw = 0.0 # 기체 정면 복귀
                print("\n[🎬 13단계] 🌐 [3인칭 전체 뷰 복귀] 180° 선회를 마치고 바둑판 정중앙(원점)으로 완벽 귀환!")
            self.c.target_x = self.home_x
            self.c.target_y = self.home_y

        # 14단계: 🕊️ [3인칭 정지 호버링] 원래 출발했던 그 자리 상공에서 가만히 호버링 (46.0s ~ 50.0s)
        elif t < 50.0:
            if self.step_idx != 14:
                self.step_idx = 14
                self.c.target_x = self.home_x
                self.c.target_y = self.home_y
                print("\n[🎬 14단계] 🕊️ [제자리 정지 호버링] 원래 출발했던 바로 그 자리 상공에서 피벗 안정화 호버링")

        # 15단계: 🛬 [사뿐 하강 착지] 원래 출발했던 그 자리에 정확히 수직 착지 & 날개 접기 (50.0s ~ 56.0s)
        elif t < 56.0:
            if self.step_idx != 15:
                self.step_idx = 15
                self.c.set_state(self.c.STATE_LANDING)
                print("\n[🎬 15단계] 🛬 [사뿐 하강 착지] 원래 출발했던 그 자리에 사뿐히 수직 착지 & 날개 등 뒤로 접기")
            if t > 53.5 and self.c.wings_deployed:
                self.c.wings_deployed = False # 날개 등 뒤로 쏙 접기

        # 시나리오 완주 ➔ 처음부터 무한 반복!
        else:
            print("\n" + "★" * 75)
            print("🎉 [스토리 완주] 3인칭 머리 숙임 ➔ 1인칭 하방 시선 ➔ 원점 복귀 풀 스토리보드 완주! 다시 무한 반복합니다.")
            print("★" * 75 + "\n")
            self.timeline = 0.0
            self.step_idx = -1

def print_ai_banner():
    print("=" * 80)
    print("      🎬 [반려 로봇새 (OpenBird-artnfull)] 공식 시네마틱 스토리보드 데모")
    print("=" * 80)
    print(" 🌟 대표님께서 직접 기획하신 13단계 풀 스토리보드가 펼쳐집니다:")
    print("   1. 앞으로 걷기 ➔ 2. 뒤로 걷기 ➔ 3. 꽈당 넘어지기 ➔ 4. 오뚝이 벌떡 기립!")
    print("   5. 모이 쪼기 ➔ 6. 앉았다 일어서기 ➔ 7. 날개 180° 전개 & VTOL 수직 이륙")
    print("   8. 전진 비행 ➔ 9. 눈 아래 55° 향하기 ➔ 10. 직접 앞을 보는 1인칭 FPV")
    print("   11. 직접 아래를 보는 1인칭 FPV ➔ 12. 3인칭 전체화면 복귀 ➔ 13. 사뿐 착지 & 날개 접기")
    print("=" * 80 + "\n")

def main():
    print_ai_banner()

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
    controller.cam_mode = 0
    director = ShowcaseDirector(controller, model, data)
    recorder = ScreenRecorder(model, data)

    def viewer_key_callback(keycode):
        if keycode in [ord('Q'), ord('q'), 256, 27]:
            print("\nAI 시뮬레이터를 종료합니다. 안녕히 가세요!")
            if recorder.is_recording:
                recorder.stop()
            sys.exit(0)
        elif keycode in [ord('C'), ord('c')]:
            controller.cam_mode = (controller.cam_mode + 1) % 4
            cam_names = [
                "🕊️ 3인칭 전체 뷰 (외부에서 로봇새 모습 관찰)",
                "📷 [1인칭 FPV] 로봇새 머리 정중앙 시점",
                "👁️ [좌측 눈 직접 시점] 로봇새의 왼쪽 눈알에서 세상을 직접 바라보는 시점!",
                "👁️ [우측 눈 직접 시점] 로봇새의 오른쪽 눈알에서 세상을 직접 바라보는 시점!"
            ]
            print(f"\n[📷 시점 전환] 현재 시점: {cam_names[controller.cam_mode]}")
        elif keycode in [ord('V'), ord('v')]:
            controller.toggle_look_down()
        elif keycode in [ord('B'), ord('b')]:
            controller.look_front()
        elif keycode in [ord('X'), ord('x')]:
            recorder.toggle(controller.cam_mode)
        elif keycode in [ord('K'), ord('k')]:
            print("\n[💥 사용자 개입] 사용자가 로봇을 툭 쳐서 넘어뜨렸습니다!")
            data.qvel[0] = 0.35
            data.qvel[3] = 4.5
            data.qvel[4] = 2.0
            controller.set_state(RobotBirdFSM.STATE_KNOCKDOWN)

    with mujoco.viewer.launch_passive(model, data, key_callback=viewer_key_callback, show_left_ui=False, show_right_ui=False) as viewer:
        try:
            viewer.cam.distance = 1.05
            viewer.cam.elevation = -10
            viewer.cam.azimuth = -80

            dt = model.opt.timestep if model.opt.timestep > 0 else 0.002

            while viewer.is_running():
                step_start = time.time()

                # 🎬 대표님의 스토리보드 디렉터 구동
                director.update(dt)

                # 🦆 물리 제어기 업데이트 및 물리 스텝 전진
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

                # 🎥 스마트 카메라 시점 동기화
                try:
                    if controller.cam_mode == 0:
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                        viewer.cam.lookat[0] = data.qpos[0]
                        viewer.cam.lookat[1] = data.qpos[1]
                        viewer.cam.lookat[2] = data.qpos[2]
                    elif controller.cam_mode == 1:
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                        viewer.cam.fixedcamid = model.camera("head_fpv_cam").id
                    elif controller.cam_mode == 2:
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                        viewer.cam.fixedcamid = model.camera("left_eye_cam").id
                    elif controller.cam_mode == 3:
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                        viewer.cam.fixedcamid = model.camera("right_eye_cam").id
                except Exception:
                    pass

                # 🔴 실시간 비디오 프레임 캡처 (녹화 중일 때 자동 기록)
                if recorder.is_recording:
                    recorder.capture_frame(controller.cam_mode)

                viewer.sync()

                elapsed = time.time() - step_start
                sleep_time = dt - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\n사용자에 의해 AI 시뮬레이터가 종료되었습니다.")
        finally:
            if recorder.is_recording:
                recorder.stop()

if __name__ == "__main__":
    main()
