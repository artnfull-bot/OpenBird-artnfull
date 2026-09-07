import os
import sys
import numpy as np
import time
import math

# UTF-8 입출력 보장
if sys.platform == "win32":
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
        sys.stderr.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass

class RobotBirdFSM:
    """
    반려 로봇새(Bipedal VTOL Robot Bird) 고성능 4단계 제어기
    - 1단계: 등 뒤 세로 접힘(사진 1) <-> 좌우 180° 수평 전개(사진 2)
    - 2단계: 프로펠러 쌩쌩 회전 가속
    - 3단계: 2.0m 고도로 똑바로 수직 상승
    - 4단계: 2.0m 완벽 정지 호버링
    """
    STATE_STAND = "STAND"
    STATE_WALK = "WALK"
    STATE_GROUND_PICK = "GROUND_PICK"
    STATE_WING_SHIVER = "WING_SHIVER"
    STATE_SIT = "SIT"
    STATE_KNOCKDOWN = "KNOCKDOWN"
    STATE_TAKEOFF_SPOOL = "TAKEOFF_SPOOL"  # 2단계: 프로펠러 회전 가속
    STATE_TAKEOFF_CLIMB = "TAKEOFF_CLIMB"  # 3단계: 수직 상승 (2.0m)
    STATE_FLIGHT = "FLIGHT"                # 4단계: 2.0m 정지 호버링
    STATE_LANDING = "LANDING"
    STATE_RECOVER = "RECOVER"

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.state = self.STATE_STAND
        self.state_timer = 0.0
        
        # 🪽 날개 전개 상태 관리
        # 0.0: 등 뒤 세로 완벽 접힘 (사진 1)
        # 1.0: 좌우 180° 수평 전개 비행 준비 (사진 2)
        self.wing_progress = 0.0
        self.wings_deployed = False
        
        # 🌪️ 프로펠러 회전 관리
        self.propeller_running = False
        self.prop_angle = 0.0
        self.prop_speed = 0.0
        self.target_prop_speed = 50.0  # 초당 약 8회전 (눈으로 역동적인 회전이 완벽히 보이는 속도)

        # 🚁 비행 조종 입력 (기본 1.0m 목표 고도)
        self.cmd_roll = 0.0
        self.cmd_pitch = 0.0
        self.cmd_yaw = 0.0
        self.target_altitude = 1.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0  # 🌟 목표 회전각 (라디안)

        # 🦆 보행 조종 입력
        self.walk_speed = 0.0
        self.walk_turn = 0.0
        self.walk_phase = 0.0

        self.total_mass = sum(self.model.body_mass)
        self.gravity_force = self.total_mass * 9.81
        self.base_thrust = self.gravity_force / 2.0

        # 🛡️ 외란 저항 및 자동 기립 감지 타이머
        self.fall_detect_timer = 0.0

        # 🥊 물리적 외란 밀기 시뮬레이션 타이머
        self.push_force = np.zeros(3)
        self.push_timer = 0.0

        # 📷 하방 관측/정찰 모드 (Look-Down Survey): 비행 중 몸통만 아래로 숙이고 날개는 수평 유지
        self.look_down = False
        self.look_down_pitch = 0.0
        self.target_look_down_pitch = 0.0

    def apply_push(self, fx=0.0, fy=0.0, duration=0.18):
        """외란 테스트: 특정 방향으로 duration 초 동안 지속적인 밀기 힘 인가"""
        self.push_force = np.array([fx, fy, 0.0])
        self.push_timer = duration

    def set_state(self, new_state):
        if self.state != new_state:
            print(f"\n[모드 전환] {self.state} -> {new_state}")
            self.state = new_state
            self.state_timer = 0.0
            if new_state in [self.STATE_FLIGHT, self.STATE_TAKEOFF_CLIMB]:
                self.target_x = self.data.qpos[0]
                self.target_y = self.data.qpos[1]

    def toggle_wings(self):
        """1단계: 등 뒤 세로 접힘(사진 1) ➔ 좌우 180° 수평 전개(사진 2) 토글 (1번 / E키)"""
        self.wings_deployed = not self.wings_deployed
        if not self.wings_deployed:
            self.propeller_running = False
            self.set_state(self.STATE_STAND)
            print("\n[날개 접기] 등 뒤로 세로로 쏙 포개어 접었습니다. (사진 1 상태)")
        else:
            print("\n[1단계 완료] 등 뒤에 접혀있던 날개를 좌우 180도 수평으로 쫙 폈습니다! (사진 2 상태)")

    def toggle_look_down(self):
        """📷 눈알 하방 시선 토글 (V 키) - 눈알 아래 55° 보기 <-> 눈알 원래 위치로 복귀"""
        self.look_down = not self.look_down
        if self.look_down:
            self.target_look_down_pitch = 0.95  # 약 55도 아래로 숙임
            print("\n[👇 눈알 아래 보기 ON] 눈알이 아래로 55° 향하여 바닥을 내려다봅니다!")
        else:
            self.target_look_down_pitch = 0.0
            print("\n[👀 눈알 원위치 복귀] 눈알이 다시 원래 위치(정면)로 돌아왔습니다! ✨")

    def look_front(self):
        """👀 원터치 눈알 원위치 복귀 (B 키)"""
        self.look_down = False
        self.target_look_down_pitch = 0.0
        print("\n[👀 눈알 원위치 복귀] 눈알이 다시 원래 위치(정면)로 돌아왔습니다! ✨")

    def start_flight_sequence(self):
        """2단계: 프로펠러 회전 ➔ 3단계 1m 수직 상승 ➔ 4단계 1m 정지 호버링 (2번 / SPACE키)"""
        if not self.wings_deployed or self.wing_progress < 0.8:
            print("\n[1단계] 등 뒤의 날개를 먼저 좌우 180도로 쫙 폅니다...")
            self.wings_deployed = True
            self.wing_progress = 1.0  # 즉시 전개 완료
        
        print("[2단계] 3엽 컬러 프로펠러가 쌩쌩 돌기 시작합니다!")
        self.propeller_running = True
        self.target_altitude = 1.0
        self.set_state(self.STATE_TAKEOFF_SPOOL)

    def update(self, dt):
        self.state_timer += dt
        
        ctrl = np.zeros(self.model.nu)
        pos = self.data.qpos[0:3]
        vel = self.data.qvel[0:3]
        z_height = pos[2]
        
        # ==================== [ 🌪️ 3엽 프로펠러 쌩쌩 회전 ] ====================
        if self.propeller_running:
            self.prop_speed = min(self.target_prop_speed, self.prop_speed + 80.0 * dt)
        else:
            self.prop_speed = max(0.0, self.prop_speed - 50.0 * dt)
        
        # 물리 엔진 DOF 속도 및 각도 동기화
        l_prop_dof = self.model.joint("left_prop_spin").dofadr[0]
        r_prop_dof = self.model.joint("right_prop_spin").dofadr[0]
        l_prop_qpos = self.model.joint("left_prop_spin").qposadr[0]
        r_prop_qpos = self.model.joint("right_prop_spin").qposadr[0]

        self.data.qvel[l_prop_dof] = self.prop_speed
        self.data.qvel[r_prop_dof] = -self.prop_speed
        
        if self.prop_speed > 0:
            self.prop_angle += self.prop_speed * dt
            self.data.qpos[l_prop_qpos] = self.prop_angle
            self.data.qpos[r_prop_qpos] = -self.prop_angle

        # ==================== [ 🪽 날개 전개/접힘 보간 애니메이션 ] ====================
        target_progress = 1.0 if self.wings_deployed else 0.0
        wing_speed = 3.5  # 약 0.28초 만에 매끄럽게 전개/접힘
        if self.wing_progress < target_progress:
            self.wing_progress = min(target_progress, self.wing_progress + wing_speed * dt)
        elif self.wing_progress > target_progress:
            self.wing_progress = max(target_progress, self.wing_progress - wing_speed * dt)

        # ==================== [ 📷 하방 관측 각도 보간 및 날개 역보상 ] ====================
        self.look_down_pitch += (self.target_look_down_pitch - self.look_down_pitch) * min(1.0, dt * 5.0)

        # fold: 0.0 (등 뒤) ➔ ±1.57 (좌우 180° 전개)
        fold_l_cmd = 1.57 * self.wing_progress
        fold_r_cmd = -1.57 * self.wing_progress

        # tilt: 1.57 (등 뒤 세로 접힘) ➔ 0.0 (비행 시 완벽한 가로 수평 눕기!)
        # 🌟 날개가 전개되면 항상 처음 비행할 때의 가로 수평 날개(0.0 rad)를 유지하고, 몸통만 아래로 숙여집니다!
        tilt_cmd = 1.57 * (1.0 - self.wing_progress)

        ctrl[6] = fold_l_cmd
        ctrl[7] = fold_r_cmd
        ctrl[8] = tilt_cmd
        ctrl[9] = tilt_cmd
        # 🐦 [독립 머리/목 피치] 몸통/날개는 완벽 수평 고정, 머리(두 눈+부리)가 아래로 55° 쏙 숙여짐!
        ctrl[10] = -self.look_down_pitch

        torso_id = self.model.body("torso").id
        fz = 0.0
        fx = 0.0
        fy = 0.0
        tau_x = 0.0
        tau_y = 0.0
        tau_z = 0.0

        # =========================================================================
        # 🦆 [ 지상 모드: 보행 및 직립 ] - 비행 물리와 100% 분리!
        # =========================================================================
        is_ground_mode = self.state in [
            self.STATE_STAND, self.STATE_WALK, self.STATE_GROUND_PICK, 
            self.STATE_WING_SHIVER, self.STATE_SIT, self.STATE_RECOVER, self.STATE_KNOCKDOWN
        ]

        if is_ground_mode:
            # 쿼터니언 기반 상체 기울기 측정 (q = [w, x, y, z] -> x축: Pitch, y축: Roll)
            q = self.data.qpos[3:7]
            q_sign = np.sign(q[0]) if q[0] != 0 else 1.0
            pitch_tilt = -q[1] * q_sign   # X축 회전 = 앞뒤 기울기 (Pitch)
            roll_tilt = -q[2] * q_sign    # Y축 회전 = 좌우 기울기 (Roll)

            # 1. 🐦 제자리 직립 모드 (STAND) - 외란 저항 (밀리지 않고 버티기 / Push Recovery)
            if self.state == self.STATE_STAND:
                # 선속도 및 각속도 피드백을 결합한 지능형 버티기 (Push Resistance)
                lin_vel = self.data.cvel[torso_id, 3:6]
                vy = lin_vel[1]
                vx = lin_vel[0]

                bal_pitch = np.clip(pitch_tilt * 1.2 - vy * 0.18, -0.35, 0.35)
                bal_roll = np.clip(roll_tilt * 0.8 - vx * 0.12, -0.20, 0.20)

                # 호흡 모션
                breath = 0.012 * math.sin(self.state_timer * 3.0)

                # 좌측 다리 (외란에 맞서 발목과 고관절로 바닥을 꽉 지탱)
                ctrl[0] = 0.0 + bal_pitch + bal_roll
                ctrl[1] = 0.0 + breath
                ctrl[2] = 0.0 - bal_pitch * 1.2

                # 우측 다리
                ctrl[3] = 0.0 + bal_pitch - bal_roll
                ctrl[4] = 0.0 + breath
                ctrl[5] = 0.0 - bal_pitch * 1.2

            # 2. 🚶 2족 보행 모드 (WALK) - 진짜 앞으로/뒤로 걷고 시원하게 회전하는 조류 2족보행
            elif self.state == self.STATE_WALK:
                # 걸음 주기 위상 진척
                self.walk_phase += dt * 6.5 * max(0.6, abs(self.walk_speed))
                
                phase_sin = math.sin(self.walk_phase)
                phase_cos = math.cos(self.walk_phase)
                dir_sign = 1.0 if self.walk_speed >= 0 else -1.0
                turn = getattr(self, 'walk_turn', 0.0)

                # 🌟 [후진 보행 100% 안정화] 후진 시 상체를 앞쪽으로 8° 확실하게 숙여 무게중심을 안정화 (넘어짐 원천 차단!)
                lean_bias = 0.14 if self.walk_speed < 0 else 0.0
                bal_pitch = np.clip((pitch_tilt + lean_bias) * 1.0, -0.30, 0.30)
                bal_roll = np.clip(roll_tilt * 0.6, -0.15, 0.15)

                # 전진 보폭 0.25, 후진 보폭 0.18 (촘촘하고 당당하게 뒤로 걷기)
                base_stride = 0.25 if self.walk_speed >= 0 else 0.18
                
                # 🔄 좌/우 회전 시 양 발의 차동 보폭 (Differential Stride) 적용 ➔ 팽그르르 즉각 회전!
                l_stride = base_stride * (1.0 - turn * 0.35) * dir_sign
                r_stride = base_stride * (1.0 + turn * 0.35) * dir_sign
                
                # 🦶 좌측 다리 (phase_sin > 0: 유각기 발 들림 / phase_sin <= 0: 지지기 바닥 밀기)
                if phase_sin > 0:
                    l_hip = l_stride * (-phase_cos) + bal_pitch + bal_roll
                    l_knee = 0.35 * phase_sin  # 무릎을 굽혀 발을 바닥에서 1.5cm 높이 띄움!
                    l_ankle = -(l_hip + l_knee) # 발바닥이 항상 지면과 수평 유지
                else:
                    l_hip = l_stride * (-phase_cos) + bal_pitch + bal_roll
                    l_knee = 0.0               # 지지 다리 무릎 펴서 지지
                    l_ankle = -l_hip           # 지면 밀착 지지
                    
                # 🦶 우측 다리 (phase_sin <= 0: 유각기 발 들림 / phase_sin > 0: 지지기 바닥 밀기)
                if phase_sin <= 0:
                    r_hip = r_stride * phase_cos + bal_pitch - bal_roll
                    r_knee = 0.35 * (-phase_sin) # 무릎 굽혀 발 띄움
                    r_ankle = -(r_hip + r_knee)  # 발바닥 수평 유지
                else:
                    r_hip = r_stride * phase_cos + bal_pitch - bal_roll
                    r_knee = 0.0                # 지지 다리 무릎 펴서 지지
                    r_ankle = -r_hip            # 지면 밀착 지지

                ctrl[0] = l_hip
                ctrl[1] = l_knee
                ctrl[2] = l_ankle

                ctrl[3] = r_hip
                ctrl[4] = r_knee
                ctrl[5] = r_ankle

                # 지지발의 지면 반력 추진력 (후진 시에도 안정된 힘 인가)
                R = self.data.xmat[torso_id].reshape(3, 3)
                y_forward = R[:, 1]
                
                push_mag = 1.1 if self.walk_speed >= 0 else 0.85
                push_force = y_forward * (push_mag * self.walk_speed)
                fx = push_force[0]
                fy = push_force[1]
                
                # 🔄 회전 보행 시 헤딩 갱신 및 강력한 Yaw 회전 토크 (좌/우 완전 대칭 회전)
                omega_ground = self.data.cvel[torso_id, 0:3]
                if abs(turn) > 0.05:
                    self.target_yaw -= turn * 1.8 * dt
                    tau_z = np.clip(-turn * 0.40 - omega_ground[2] * 0.10, -0.32, 0.32)
                else:
                    target_yaw = getattr(self, 'target_yaw', 0.0)
                    current_yaw = math.atan2(-y_forward[0], y_forward[1])
                    yaw_err = (target_yaw - current_yaw + math.pi) % (2 * math.pi) - math.pi
                    tau_z = np.clip(yaw_err * 0.50 - omega_ground[2] * 0.12, -0.20, 0.20)

            # 3. 🌾 감성 모션들 (조류 역관절 특화)
            elif self.state == self.STATE_GROUND_PICK:
                t = self.state_timer
                # 🌟 [비행식 다리 깊은 접힘 ➔ 부리 바닥 터치 3단 모이 쪼기]
                # 1단계 (0~0.42s): 다리를 몸 밑으로 깊게 웅크려 접으며 부리가 바닥 지면에 콕! 닿음
                # 2단계 (0.42~0.85s): 바닥에서 콕! 콕! 쪼기
                # 3단계 (0.85~1.30s): 부드럽게 원래 서 있던 기립 자세로 슥- 복귀!
                if t < 0.42:
                    p = 0.5 * (1.0 - math.cos(math.pi * t / 0.42))  # 0 ➔ 1 깊은 하강
                    hip = 0.65 * p
                    knee = 0.95 * p
                    ankle = -0.48 * p
                elif t < 0.85:
                    mini = 0.14 * math.sin((t - 0.42) * 18.0)       # 바닥 콕! 콕!
                    hip = 0.65 + mini
                    knee = 0.95
                    ankle = -0.48
                elif t < 1.30:
                    p = 0.5 * (1.0 + math.cos(math.pi * (t - 0.85) / 0.45))  # 1 ➔ 0 기립 복귀
                    hip = 0.65 * p
                    knee = 0.95 * p
                    ankle = -0.48 * p
                else:
                    hip = 0.0; knee = 0.0; ankle = 0.0

                ctrl[0] = hip; ctrl[1] = knee; ctrl[2] = ankle
                ctrl[3] = hip; ctrl[4] = knee; ctrl[5] = ankle

                # 🌟 회전 억제 및 반동 0% 댐핑
                omega_ground = self.data.cvel[torso_id, 0:3]
                tau_z = np.clip(-omega_ground[2] * 0.30, -0.15, 0.15)
                tau_x = 0.0; tau_y = 0.0

                if t >= 1.30:
                    self.set_state(self.STATE_STAND)

            elif self.state == self.STATE_KNOCKDOWN:
                # 💥 꽈당 누워있는 수동 테스트 상태: 힘을 빼고 편안하게 지면에 누워있음 (일어나려면 'P' 키 누름)
                ctrl[0] = 0.0; ctrl[1] = 0.20; ctrl[2] = 0.0
                ctrl[3] = 0.0; ctrl[4] = 0.20; ctrl[5] = 0.0
                tau_x = 0.0; tau_y = 0.0; tau_z = 0.0

            elif self.state == self.STATE_WING_SHIVER:
                t = self.state_timer
                if t < 1.5:
                    shiver = 0.25 * math.sin(t * 28.0)
                    ctrl[0] = 0.0; ctrl[1] = 0.0; ctrl[2] = 0.0
                    ctrl[3] = 0.0; ctrl[4] = 0.0; ctrl[5] = 0.0
                    ctrl[6] = max(0, shiver); ctrl[7] = -max(0, shiver)
                else:
                    self.set_state(self.STATE_STAND)

            elif self.state == self.STATE_SIT:
                # 🐥 완벽하게 무릎을 굽히고 다리를 웅크려 쪼그려 앉기
                ctrl[0] = -0.40; ctrl[1] = 0.90; ctrl[2] = -0.50
                ctrl[3] = -0.40; ctrl[4] = 0.90; ctrl[5] = -0.50

            elif self.state == self.STATE_RECOVER:
                t = self.state_timer
                # 🌟 [100% 벌떡 일어나는 3D 오뚝이 기립]
                # 어떤 각도로 넘어져 있어도 바닥 마찰을 털어내고 수직으로 우뚝 일어섬!
                R = self.data.xmat[torso_id].reshape(3, 3)
                z_body = R[:, 2]
                z_world = np.array([0.0, 0.0, 1.0])
                tau_tilt = np.cross(z_body, z_world) # 3차원 수직 정렬 토크
                omega_ground = self.data.cvel[torso_id, 0:3]

                ctrl[0] = 0.0; ctrl[1] = 0.0; ctrl[2] = 0.0
                ctrl[3] = 0.0; ctrl[4] = 0.0; ctrl[5] = 0.0

                # 0~0.25초: 바닥 마찰 해제를 위한 수직 팝업 리프트 인가
                fz = 3.2 if t < 0.25 else 0.0

                # 강력한 수직 직립 토크 및 스핀 억제
                tau_x = np.clip(tau_tilt[0] * 5.0 - omega_ground[0] * 0.30, -0.60, 0.60)
                tau_y = np.clip(tau_tilt[1] * 5.0 - omega_ground[1] * 0.30, -0.60, 0.60)
                tau_z = np.clip(-omega_ground[2] * 0.35, -0.20, 0.20)

                # 수직 정렬이 완료되면 STAND로 복귀
                z_err = 1.0 - z_body[2]
                if t > 0.40 and z_err < 0.03:
                    self.set_state(self.STATE_STAND)
                elif t > 1.20:
                    self.set_state(self.STATE_STAND)

            # 🚨 [지능형 자동 넘어짐 감지 및 즉시 오뚝이 기립 (Auto Knockdown Recovery)]
            R_body = self.data.xmat[torso_id].reshape(3, 3)
            z_up = R_body[2, 2] # 1.0 = 똑바로 직립, 0.0 = 90도 쓰러짐
            torso_z = self.data.qpos[2]

            # 정상 직립이 아닐 때 (기울어짐 > 50° 또는 높이 < 6.5cm)
            if self.state not in [self.STATE_RECOVER, self.STATE_KNOCKDOWN, self.STATE_GROUND_PICK, self.STATE_SIT]:
                if z_up < 0.60 or torso_z < 0.065:
                    self.fall_detect_timer += dt
                    if self.fall_detect_timer >= 0.20:
                        print(f"\n[오뚝이 감지] 로봇새 넘어짐 감지! (기울기 z_up={z_up:.2f}, 높이={torso_z*100:.1f}cm) -> 자동으로 벌떡 일어납니다!")
                        self.fall_detect_timer = 0.0
                        self.set_state(self.STATE_RECOVER)
                else:
                    self.fall_detect_timer = max(0.0, self.fall_detect_timer - dt * 2.0)

            # 지상 모드 상체 자세 유지 댐핑 (넘어짐/기립 상태 외 일반 보행/기립 시)
            if self.state not in [self.STATE_RECOVER, self.STATE_KNOCKDOWN, self.STATE_GROUND_PICK]:
                omega_ground = self.data.cvel[torso_id, 0:3]
                lean_bias = -0.05 if (self.state == self.STATE_WALK and self.walk_speed < 0) else 0.0
                tau_x = np.clip((pitch_tilt + lean_bias) * 3.5 - omega_ground[0] * 0.22, -0.35, 0.35)
                tau_y = np.clip(roll_tilt * 2.5 - omega_ground[1] * 0.15, -0.25, 0.25)
                # 🌟 보행 중이 아닐 때만 제자리 회전 억제, 보행 중(WALK)일 때는 회전 토크 보존!
                if self.state != self.STATE_WALK:
                    tau_z = np.clip(-omega_ground[2] * 0.25, -0.15, 0.15)



        # =========================================================================
        # 🚁 [ 비행 모드: 이륙, 호버링, 착륙 ] - 2번 키 누를 때만 작동!
        # =========================================================================
        else:
            # 월드 회전 행렬 및 월드 각속도 (cvel: world-frame angular velocity)
            R = self.data.xmat[torso_id].reshape(3, 3)
            omega_world = self.data.cvel[torso_id, 0:3]
            y_body = R[:, 1]
            z_body = R[:, 2]

            # 1. 🌟 수평 유지 (Roll / Pitch Leveling) - 월드 Z축 완벽 수직 정렬 (프로펠러와 몸통은 항상 가로 수평 유지!)
            z_world = np.array([0.0, 0.0, 1.0])
            tau_tilt = np.cross(z_body, z_world)  # [tilt_x, tilt_y, 0] (순수 수평 복원력)

            # 2. 🌟 목표 헤딩 추종 (Yaw Heading) - 최단 각도 오차 계산 (-pi ~ +pi)
            current_yaw = math.atan2(-y_body[0], y_body[1])
            yaw_err = (self.target_yaw - current_yaw + math.pi) % (2 * math.pi) - math.pi

            # 3. 🌟 3축 직교 독립 안정화 제어 (Cross-coupling 100% 원천 차단!)
            # Roll / Pitch: 강한 복원력과 댐핑으로 기우뚱 방지
            tau_x = np.clip(tau_tilt[0] * 3.5 - omega_world[0] * 0.35, -0.4, 0.4)
            tau_y = np.clip(tau_tilt[1] * 3.5 - omega_world[1] * 0.35, -0.4, 0.4)
            # Yaw: 부드럽고 정확한 헤딩 추종 및 댐핑
            tau_z = np.clip(yaw_err * 0.50 - omega_world[2] * 0.12, -0.20, 0.20)

            # 🌟 [비행 모드 다리 접힘]
            # 비행기 랜딩기어 및 새의 비행 자세처럼 다리는 몸통 아래로 쏙 접어 올리고,
            # 발바닥은 완벽하게 가로 수평(지면 평행)을 유지하여 몸에 단정하게 착 붙입니다!
            if self.state in [self.STATE_TAKEOFF_CLIMB, self.STATE_FLIGHT]:
                ctrl[0] = -0.38; ctrl[1] = 0.80; ctrl[2] = -0.42
                ctrl[3] = -0.38; ctrl[4] = 0.80; ctrl[5] = -0.42
            elif self.state == self.STATE_LANDING:
                # 착륙 시 지상 25cm 이하로 내려오면 다리를 부드럽게 펴서 착지 준비!
                if z_height > 0.25:
                    ctrl[0] = -0.38; ctrl[1] = 0.80; ctrl[2] = -0.42
                    ctrl[3] = -0.38; ctrl[4] = 0.80; ctrl[5] = -0.42
                else:
                    ctrl[0] = 0.0; ctrl[1] = 0.0; ctrl[2] = 0.0
                    ctrl[3] = 0.0; ctrl[4] = 0.0; ctrl[5] = 0.0
            else:
                ctrl[0] = 0.0; ctrl[1] = 0.0; ctrl[2] = 0.0
                ctrl[3] = 0.0; ctrl[4] = 0.0; ctrl[5] = 0.0

            # 1. 이륙 준비 (프로펠러 가속)
            if self.state == self.STATE_TAKEOFF_SPOOL:
                # 현재 위치 앵커 캡처
                self.target_x = pos[0]
                self.target_y = pos[1]
                
                # 현재 yaw 각도 캡처
                siny_cosp = 2 * (self.data.qpos[3] * self.data.qpos[6] + self.data.qpos[4] * self.data.qpos[5])
                cosy_cosp = 1 - 2 * (self.data.qpos[5] * self.data.qpos[5] + self.data.qpos[6] * self.data.qpos[6])
                self.target_yaw = math.atan2(siny_cosp, cosy_cosp)

                if self.state_timer >= 0.35:
                    print(f"[3단계] 1.0m 높이로 똑바로 수직 상승합니다...")
                    self.set_state(self.STATE_TAKEOFF_CLIMB)

            # 2. 1.0m 수직 상승
            elif self.state == self.STATE_TAKEOFF_CLIMB:
                # 수직 상승 추력
                err_z = self.target_altitude - z_height
                fz = self.gravity_force + np.clip(err_z * 8.0 - vel[2] * 3.5, -self.gravity_force * 0.4, self.gravity_force * 1.6)

                # 수평 위치 고정
                err_x = self.target_x - pos[0]
                err_y = self.target_y - pos[1]
                fx = np.clip(err_x * 4.0 - vel[0] * 3.0, -1.0, 1.0)
                fy = np.clip(err_y * 4.0 - vel[1] * 3.0, -1.0, 1.0)

                if z_height >= self.target_altitude - 0.05 or self.state_timer > 1.8:
                    print(f"[4단계 완료] 1.0m 높이에 도달하여 완벽히 정지 호버링합니다!")
                    self.set_state(self.STATE_FLIGHT)

            # 3. 1.0m 완벽 정지 호버링 및 자유 비행 (서있을 때의 단정한 발 모양 유지)
            elif self.state == self.STATE_FLIGHT:
                # 고도 유지 (Z Altitude Hold)
                err_z = self.target_altitude - z_height
                fz = self.gravity_force + np.clip(err_z * 8.5 - vel[2] * 3.5, -self.gravity_force * 0.5, self.gravity_force * 1.5)

                # 수평 위치 이동 및 추종 (X, Y Position Flight Control) - 시원하고 민첩한 비행!
                err_x = self.target_x - pos[0]
                err_y = self.target_y - pos[1]
                fx = np.clip(err_x * 8.0 - vel[0] * 3.5, -2.5, 2.5)
                fy = np.clip(err_y * 8.0 - vel[1] * 3.5, -2.5, 2.5)

            # 4. 수직 착륙
            elif self.state == self.STATE_LANDING:
                if z_height > 0.15:
                    fz = self.gravity_force * 0.70
                    err_x = self.target_x - pos[0]
                    err_y = self.target_y - pos[1]
                    fx = np.clip(err_x * 4.0 - vel[0] * 2.5, -1.0, 1.0)
                    fy = np.clip(err_y * 4.0 - vel[1] * 2.5, -1.0, 1.0)
                else:
                    print("[착륙 완료] 지면에 안전하게 착지했습니다. 프로펠러를 끄고 날개를 등 뒤로 접습니다.")
                    fz = 0.0
                    self.propeller_running = False
                    self.wings_deployed = False
                    self.set_state(self.STATE_STAND)

        self.data.ctrl[:] = ctrl

        # 🥊 외란 밀기 힘 인가 (테스트 키 X, Z 누를 때)
        if self.push_timer > 0.0:
            fx += self.push_force[0]
            fy += self.push_force[1]
            self.push_timer -= dt

        # 외력 및 토크 인가 (지상/비행 안전 클램핑)
        self.data.xfrc_applied[:] = 0.0
        if fz > 0.0 or abs(fy) > 0.0 or abs(fx) > 0.0 or abs(tau_x) > 0.0 or abs(tau_y) > 0.0 or abs(tau_z) > 0.0:
            self.data.xfrc_applied[torso_id, :3] = np.array([fx, fy, fz])
            self.data.xfrc_applied[torso_id, 3:6] = np.array([tau_x, tau_y, tau_z])



