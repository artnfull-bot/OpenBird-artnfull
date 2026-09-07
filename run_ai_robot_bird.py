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

def print_ai_banner():
    print("=" * 80)
    print("      🧠 [반려 로봇새 (Bipedal VTOL Robot Bird)] AI 자율 지능 두뇌 모드")
    print("=" * 80)
    print(" 🌟 로봇새가 스스로 호기심, 감정, 에너지 상태를 바탕으로 자율 행동을 결정합니다:")
    print("   1. 🌾 모이 쪼기     : 바닥의 먹이를 발견하고 다리를 접어 콕! 콕! 콕!")
    print("   2. 🚶 아장아장 걷기 : 주변을 탐색하며 앞/뒤 걷기 및 방향 전환")
    print("   3. 🪽 자율 비행     : 날개를 쫙 펴고 1.0m 이륙하여 공중 순항 비행")
    print("   4. 🛬 사뿐 착륙     : 안전하게 지상에 착지 후 쪼그려 앉아 휴식")
    print("   5. 🔄 자율 오뚝이   : 외부 충격으로 넘어지면 스스로 감지하여 100% 벌떡 기립!")
    print("   ─────────────────────────────────────────────────────────────")
    print(" 💡 [사용자 수동 개입 & 편의 기능]")
    print("   - [ C ] 키 : 📷 카메라 시점 전환 (3인칭 ➔ 전방 FPV ➔ 좌측 눈 ➔ 우측 눈)")
    print("   - [ V / B ] 키 : 👀 눈알 시선 조작 (👇 눈알 아래 55° 보기(V) 🔁 다시 눈알 원래 위치로 복귀(V/B))")
    print("   - [ X ] 키 : 🔴 실시간 화면 표준 H.264 MP4 동영상 녹화 ON / OFF")
    print("   - [ K ] 키 : 💥 로봇새 꽈당 넘어뜨리기 (AI가 스스로 판단해서 일어남)")
    print("   - [ Q / ESC ] : 시뮬레이터 종료")
    print("=" * 80)
    print("\nAI 로봇새 시뮬레이터를 시작합니다... (로봇새의 귀여운 자율 행동을 관찰해보세요!)\n")

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
    ai_brain = RobotBirdAIBrain(controller)
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
            viewer.cam.distance = 1.6
            viewer.cam.elevation = -15
            viewer.cam.azimuth = 145

            dt = model.opt.timestep if model.opt.timestep > 0 else 0.002

            while viewer.is_running():
                step_start = time.time()

                # 🧠 AI 자율 두뇌 업데이트
                ai_brain.update(dt)

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
