import time
import random
import math
import numpy as np

class RobotBirdAIBrain:
    """
    반려 로봇새(Bipedal VTOL Robot Bird) 자율 지능 두뇌 (Autonomous AI Brain)
    
    [핵심 자율 행동 사이클]
    1. 🔍 호기심 탐색 (Look Around & Stand) : 주변을 갸우뚱 살피며 호기심 표현
    2. 🚶 자율 2족 보행 (Walk & Roam)       : 전진 및 방향 전환하며 아장아장 걷기
    3. 🌾 모이 쪼기 (Feed Pecking)          : 바닥의 먹이를 발견하고 다리를 접어 콕! 콕! 쪼기
    4. 🪽 비행 탐험 (VTOL Cruise)           : 날개 펴고 1m 상공으로 이륙 ➔ 공중 순항 비행
    5. 🛬 착륙 및 휴식 (Land & Crouch Sit)   : 사뿐히 착륙 후 쪼그려 앉아 편안히 휴식
    6. 💃 재롱 댄스 (Happy Dance)           : 기분이 좋아 가볍게 점프 재롱 댄스
    7. 🛡️ 외란 대응 & 자율 기립              : 넘어지면 스스로 감지하여 100% 벌떡 기립!
    """

    BEHAVIOR_LOOK_AROUND = "LOOK_AROUND"
    BEHAVIOR_WALK = "WALK"
    BEHAVIOR_PECK = "PECK"
    BEHAVIOR_FLIGHT = "FLIGHT"
    BEHAVIOR_SIT_REST = "SIT_REST"
    BEHAVIOR_DANCE = "DANCE"

    def __init__(self, controller):
        self.controller = controller
        
        # 내부 감정 및 생체 상태 파라미터 (0.0 ~ 1.0)
        self.curiosity = 0.8     # 호기심 수치 (높으면 탐색 및 비행 선호)
        self.energy = 0.9        # 에너지 수치 (낮아지면 쪼그려 앉아 휴식)
        self.happiness = 0.85    # 행복도 (높으면 재롱 댄스 및 모이 쪼기)
        
        self.current_behavior = self.BEHAVIOR_LOOK_AROUND
        self.behavior_timer = 0.0
        self.behavior_duration = 3.0
        
        # 비행 서브 단계 관리
        self.flight_substate = "NONE"
        self.flight_step_timer = 0.0

        print("\n[AI 자율 두뇌] 반려 로봇새의 인공지능 자율 행동 시스템이 가동되었습니다!")

    def update(self, dt):
        self.behavior_timer += dt
        
        # 1. 넘어져 있는 상태 감지 (로봇이 넘어져 있으면 최우선으로 기립 판단)
        if self.controller.state == self.controller.STATE_KNOCKDOWN:
            if self.behavior_timer > 0.8:
                print("\n[AI 생각] '앗! 넘어졌네? 오뚝이처럼 스스로 일어나야지!'")
                self.controller.set_state(self.controller.STATE_RECOVER)
                self.set_behavior(self.BEHAVIOR_LOOK_AROUND, duration=2.5)
            return

        # 2. 기립 복구 중이거나 모이 쪼기 중일 때는 해당 모션이 끝날 때까지 대기
        if self.controller.state in [self.controller.STATE_RECOVER, self.controller.STATE_GROUND_PICK]:
            return

        # 3. 비행 중일 때의 자율 비행 시퀀스 처리
        if self.controller.state in [
            self.controller.STATE_TAKEOFF_SPOOL,
            self.controller.STATE_TAKEOFF_CLIMB,
            self.controller.STATE_FLIGHT,
            self.controller.STATE_LANDING
        ]:
            self._handle_autonomous_flight(dt)
            return

        # 4. 현재 지상 자율 행동 지속 시간이 끝나면 다음 행동을 지능적으로 선택
        if self.behavior_timer >= self.behavior_duration:
            self._select_next_behavior()

    def set_behavior(self, behavior, duration):
        self.current_behavior = behavior
        self.behavior_timer = 0.0
        self.behavior_duration = duration

    def _select_next_behavior(self):
        """감정 및 에너지 상태에 기반한 확률적 자율 행동 선택 엔진"""
        r = random.random()

        # 에너지가 낮으면 휴식(앉기) 선택 확률 증가
        if self.energy < 0.4 and self.controller.state != self.controller.STATE_SIT:
            print("\n[AI 생각] '조금 피곤해졌어.. 다리를 쏙 접고 잠시 앉아서 쉬어야지.' (휴식 모드)")
            self.controller.set_state(self.controller.STATE_SIT)
            self.set_behavior(self.BEHAVIOR_SIT_REST, duration=random.uniform(4.0, 7.0))
            self.energy = min(1.0, self.energy + 0.4)
            return

        # 앉아있던 상태라면 일어서기
        if self.controller.state == self.controller.STATE_SIT:
            print("\n[AI 생각] '푹 쉬었으니 기운차게 다시 일어나볼까?'")
            self.controller.set_state(self.controller.STATE_STAND)
            self.set_behavior(self.BEHAVIOR_LOOK_AROUND, duration=2.5)
            return

        # 행동 선택 확률 분포
        # 1) 모이 쪼기 (25%)
        # 2) 2족 보행 탐색 (30%)
        # 3) 자율 비행 (25%)
        # 4) 두리번거리기 / 재롱 댄스 (20%)
        if r < 0.25:
            # 모이 쪼기
            print("\n[AI 생각] '어! 바닥에 맛있는 모이가 있네? 콕! 콕! 쪼아먹어야지!' (모이 쪼기)")
            self.controller.set_state(self.controller.STATE_GROUND_PICK)
            self.set_behavior(self.BEHAVIOR_PECK, duration=2.5)
            self.happiness = min(1.0, self.happiness + 0.15)
            self.energy = max(0.1, self.energy - 0.05)

        elif r < 0.55:
            # 자율 2족 보행 (전진 또는 방향 전환)
            walk_types = ["forward", "turn_left", "turn_right", "backward"]
            chosen_type = random.choice(walk_types)
            dur = random.uniform(2.5, 4.5)
            
            if chosen_type == "forward":
                print(f"\n[AI 생각] '저 앞쪽에는 뭐가 있을까? 아장아장 걸어가보자!' (전진 보행 +{dur:.1f}초)")
                self.controller.walk_speed = 1.0
                self.controller.walk_turn = 0.0
            elif chosen_type == "turn_left":
                print(f"\n[AI 생각] '왼쪽에서 재미있는 소리가 들려! 왼쪽으로 돌아볼래!' (좌회전 +{dur:.1f}초)")
                self.controller.walk_speed = 0.5
                self.controller.walk_turn = 0.8
            elif chosen_type == "turn_right":
                print(f"\n[AI 생각] '오른쪽 구경을 해볼까? 오른쪽으로 돌면서 걷기!' (우회전 +{dur:.1f}초)")
                self.controller.walk_speed = 0.5
                self.controller.walk_turn = -0.8
            else:
                print(f"\n[AI 생각] '조심조심 뒤로 걸어가볼까?' (후진 보행 +{dur:.1f}초)")
                self.controller.walk_speed = -0.7
                self.controller.walk_turn = 0.0

            self.controller.set_state(self.controller.STATE_WALK)
            self.set_behavior(self.BEHAVIOR_WALK, duration=dur)
            self.energy = max(0.1, self.energy - 0.08)

        elif r < 0.80:
            # 자율 비행 시퀀스 시작
            print("\n[AI 생각] '날개를 활짝 펴고 하늘로 슝~ 날아올라 볼래!' (자율 비행 시작)")
            self.controller.toggle_wings()
            time.sleep(0.1)
            self.controller.start_flight_sequence()
            self.flight_substate = "CLIMBING"
            self.flight_step_timer = 0.0
            self.set_behavior(self.BEHAVIOR_FLIGHT, duration=15.0)
            self.energy = max(0.1, self.energy - 0.20)
            self.happiness = min(1.0, self.happiness + 0.25)

        else:
            # 두리번거리기 또는 재롱 댄스
            if random.random() < 0.5:
                print("\n[AI 생각] '기분 최고야! 늴리리야 점프 재롱 댄스!'")
                self.controller.set_state(self.controller.STATE_RECOVER)
                self.set_behavior(self.BEHAVIOR_DANCE, duration=2.5)
            else:
                print("\n[AI 생각] '주변을 두리번두리번 갸우뚱~'")
                self.controller.walk_speed = 0.0
                self.controller.walk_turn = 0.0
                self.controller.set_state(self.controller.STATE_STAND)
                self.set_behavior(self.BEHAVIOR_LOOK_AROUND, duration=random.uniform(2.0, 3.5))

    def _handle_autonomous_flight(self, dt):
        """자율 비행 중 공중 기동 및 순항 시퀀스 관리"""
        self.flight_step_timer += dt

        # 1단계: 이륙 상승 완료 대기
        if self.controller.state == self.controller.STATE_FLIGHT:
            if self.flight_substate == "CLIMBING":
                print("\n[AI 생각] '1m 상공 도달 완료! 공중에서 자유롭게 비행 탐험을 시작할게!'")
                self.flight_substate = "CRUISING"
                self.flight_step_timer = 0.0
                
            elif self.flight_substate == "CRUISING":
                # 매 2~3초마다 자율 비행 방향 변경
                if self.flight_step_timer >= 2.5:
                    self.flight_step_timer = 0.0
                    action = random.choice(["forward", "turn_left", "turn_right", "look_down", "hover"])
                    yaw = getattr(self.controller, 'target_yaw', 0.0)
                    
                    if action == "look_down":
                        print("\n[AI 생각] '어라? 저 아래 바닥에 뭐가 있지? 날개는 뜬 채로 몸통만 아래로 숙여서 카메라로 관측해보자!' (📷 하방 관측 모드 ON)")
                        self.controller.target_look_down_pitch = 0.95
                        self.controller.look_down = True
                    else:
                        if self.controller.look_down:
                            print("\n[AI 생각] '바닥 스캔 완료! 몸통을 다시 정면 수평으로 들고 비행할게!' (📷 하방 관측 모드 OFF)")
                            self.controller.target_look_down_pitch = 0.0
                            self.controller.look_down = False

                        if action == "forward":
                            print("[AI 생각] '앞으로 시원하게 활공 전진 비행!'")
                            self.controller.target_x += 0.45 * (-math.sin(yaw))
                            self.controller.target_y += 0.45 * math.cos(yaw)
                        elif action == "turn_left":
                            print("[AI 생각] '왼쪽으로 멋지게 뱅크 턴!'")
                            self.controller.target_yaw -= 0.45
                        elif action == "turn_right":
                            print("[AI 생각] '오른쪽으로 멋지게 뱅크 턴!'")
                            self.controller.target_yaw += 0.45
                        else:
                            print("[AI 생각] '공중에 가만히 떠서 여유롭게 호버링~'")

                # 일정 시간 비행 후 안전 착륙 결정
                if self.behavior_timer >= self.behavior_duration - 2.0:
                    print("\n[AI 생각] '비행 탐험 끝! 이제 지면으로 사뿐히 착륙할게!'")
                    self.controller.target_look_down_pitch = 0.0
                    self.controller.look_down = False
                    self.controller.set_state(self.controller.STATE_LANDING)
                    self.flight_substate = "LANDING"
