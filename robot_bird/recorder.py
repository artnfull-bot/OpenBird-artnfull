import os
import sys
import time
import datetime
import mujoco
import numpy as np

# UTF-8 입출력 보장
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

# 비디오 인코더 안전 임포트 (imageio 우선, cv2 대체)
try:
    import imageio
    HAS_IMAGEIO = True
except ImportError:
    imageio = None
    HAS_IMAGEIO = False

try:
    import cv2
    HAS_OPENCV = True
except ImportError:
    cv2 = None
    HAS_OPENCV = False

class ScreenRecorder:
    """
    반려 로봇새(Bipedal VTOL Robot Bird) 실시간 표준 H.264 MP4 비디오 녹화기
    - N 또는 X 키를 눌러 실시간 녹화 시작 및 중지 (Toggle)
    - 표준 H.264 (libx264, yuv420p) 적용으로 윈도우 미디어 플레이어, 맥, 스마트폰 100% 즉시 재생
    - 3인칭 스마트 뷰 및 C 키로 전환된 1인칭 눈 카메라 뷰 완벽 녹화
    - recordings/ 폴더에 날짜/시간별 MP4 자동 저장
    """
    def __init__(self, model, data, width=640, height=480, fps=30):
        self.model = model
        self.data = data
        self.width = width
        self.height = height
        self.fps = fps
        self.renderer = mujoco.Renderer(model, height, width)
        self.orbit_cam = mujoco.MjvCamera()
        self.orbit_cam.azimuth = 145
        self.orbit_cam.elevation = -15
        self.orbit_cam.distance = 1.6
        
        self.is_recording = False
        self.writer_type = None  # 'imageio' or 'cv2'
        self.video_writer = None
        self.output_filepath = ""
        self.frame_count = 0
        self.start_time = 0.0
        self.last_capture_time = 0.0
        self.frame_interval = 1.0 / self.fps

        self.recordings_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "recordings")
        os.makedirs(self.recordings_dir, exist_ok=True)

    def toggle(self, cam_mode=0):
        if not self.is_recording:
            self.start(cam_mode)
        else:
            self.stop()

    def start(self, cam_mode=0):
        if not HAS_IMAGEIO and not HAS_OPENCV:
            print("\n" + "=" * 70)
            print(" ⚠️ [녹화 모듈 안내] 비디오 녹화를 위해 imageio 또는 opencv-python 패키지가 필요합니다.")
            print(" 💡 설치 명령어: pip install imageio[ffmpeg] opencv-python")
            print("=" * 70 + "\n")
            return

        now_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        cam_tag = "3rd" if cam_mode == 0 else ("fpv" if cam_mode == 1 else ("lefteye" if cam_mode == 2 else "righteye"))
        filename = f"robot_bird_{cam_tag}_{now_str}.mp4"
        self.output_filepath = os.path.join(self.recordings_dir, filename)

        # 🌟 표준 H.264 (libx264, yuv420p) 인코더로 윈도우 미디어 플레이어 100% 호환 보장
        if HAS_IMAGEIO:
            try:
                self.video_writer = imageio.get_writer(
                    self.output_filepath, 
                    fps=self.fps, 
                    codec='libx264', 
                    pixelformat='yuv420p',
                    format='FFMPEG'
                )
                self.writer_type = 'imageio'
            except Exception:
                self.writer_type = None

        if self.video_writer is None and HAS_OPENCV:
            try:
                # FourCC H264 / mp4v fallback
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                self.video_writer = cv2.VideoWriter(self.output_filepath, fourcc, self.fps, (self.width, self.height))
                self.writer_type = 'cv2'
            except Exception:
                self.writer_type = None

        if self.video_writer is None:
            print(f"\n[오류] 비디오 인코더를 초기화할 수 없습니다.")
            return

        self.is_recording = True
        self.frame_count = 0
        self.start_time = time.time()
        self.last_capture_time = 0.0

        print(f"\n" + "=" * 70)
        print(f" 🔴 [REC 녹화 시작!] 표준 H.264 고화질 비디오 녹화 중... (30 FPS)")
        print(f" 📁 저장 대상: {self.output_filepath}")
        print(f" 💡 녹화를 끝내려면 다시 'X' 키를 누르세요!")
        print("=" * 70 + "\n")

    def capture_frame(self, cam_mode=0):
        if not self.is_recording or self.video_writer is None:
            return

        now = time.time()
        if now - self.last_capture_time < self.frame_interval:
            return

        self.last_capture_time = now

        try:
            if cam_mode == 0:
                self.orbit_cam.lookat[0] = self.data.qpos[0]
                self.orbit_cam.lookat[1] = self.data.qpos[1]
                self.orbit_cam.lookat[2] = self.data.qpos[2]
                self.renderer.update_scene(self.data, camera=self.orbit_cam)
            elif cam_mode == 1:
                self.renderer.update_scene(self.data, camera="head_fpv_cam")
            elif cam_mode == 2:
                self.renderer.update_scene(self.data, camera="left_eye_cam")
            elif cam_mode == 3:
                self.renderer.update_scene(self.data, camera="right_eye_cam")
            else:
                self.renderer.update_scene(self.data)

            rgb_frame = self.renderer.render()

            if self.writer_type == 'imageio':
                self.video_writer.append_data(rgb_frame)
            elif self.writer_type == 'cv2':
                bgr_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
                self.video_writer.write(bgr_frame)

            self.frame_count += 1
        except Exception as e:
            pass

    def stop(self):
        if not self.is_recording:
            return

        self.is_recording = False
        duration = time.time() - self.start_time

        try:
            if self.writer_type == 'imageio' and self.video_writer is not None:
                self.video_writer.close()
            elif self.writer_type == 'cv2' and self.video_writer is not None:
                self.video_writer.release()
        except Exception:
            pass

        self.video_writer = None
        self.writer_type = None

        file_size_mb = 0.0
        if os.path.exists(self.output_filepath):
            file_size_mb = os.path.getsize(self.output_filepath) / (1024 * 1024)

        print(f"\n" + "=" * 70)
        print(f" 💾 [녹화 완료 및 저장!] 영상이 다운로드(저장)되었습니다.")
        print(f" 📂 파일 위치: {self.output_filepath}")
        print(f" ⏱️ 녹화 시간: {duration:.1f}초 ({self.frame_count} 프레임, {file_size_mb:.2f} MB)")
        print("=" * 70 + "\n")
