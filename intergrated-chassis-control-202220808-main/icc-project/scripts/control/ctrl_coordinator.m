function actuatorCmd = ctrl_coordinator(latCmd, lonCmd, verCmd, vx, VEH, CTRL, LIM)
%CTRL_COORDINATOR [학생 작성] Actuator Allocation — 횡/종/수직 명령을 actuator 로 분배
%
%   상위 제어기들의 명령 (yaw moment, Fx_total, damping) 을 차량 actuator
%   (steerAngle, 4-wheel brake torque, 4-wheel damping) 로 변환.
%
%   Inputs:
%       latCmd.steerAngle - AFS 보조 조향 [rad]
%       latCmd.yawMoment  - ESC 요청 yaw moment [Nm]
%       lonCmd.Fx_total   - 종방향 힘 요구 [N]
%       lonCmd.brakeRatio - 제동 비율
%       verCmd            - 4×1 damping [Ns/m] (ctrl_vertical 출력)
%       vx, VEH, CTRL, LIM
%
%   Output:
%       actuatorCmd.steerAngle    - 최종 조향각 [rad], LIM.MAX_STEER_ANGLE 제한
%       actuatorCmd.brakeTorque   - 4×1 brake torque [Nm], [FL; FR; RL; RR], LIM.MAX_BRAKE_TRQ 제한
%       actuatorCmd.dampingCoeff  - 4×1 [Ns/m]
%
%   요구사항:
%       1. 종방향 제동 (lonCmd.Fx_total < 0) 의 4륜 균등 분배 — 전후 비율 60:40 권장
%       2. ESC yaw moment → brake 차동 분배 (좌/우 비대칭)
%             양의 M_z (CCW) → 좌측 brake 증가 또는 우측 brake 감소
%             track 반거리: t_f/2 = VEH.track_f/2,  t_r/2 = VEH.track_r/2
%             dT_f = M_z · ratio_f / t_f,  dT_r = M_z · (1-ratio_f) / t_r
%       3. AFS steerAngle 그대로 통과 + saturation
%       4. brake torque 합산 후 [0, MAX_BRAKE_TRQ] 클리핑
%
%   가산점 (선택):
%       - 마찰원 제한: 각 휠의 brake torque + cornering force 가 μ·Fz 안으로
%       - WLS allocation: actuator effort minimize 목적함수
%       - per-wheel 최대 토크 제한 — wheel slip 임계 도달 시 감소
%
%   힌트:
%       - half-track: t_f/2 ≈ 0.78 m (BMW_5)
%       - 종방향 brake 시 force-to-torque: T = |Fx_total|/4 · r_w  (r_w ≈ 0.33 m)
%       - allocation matrix form 도 가능 (LQ allocation)

    %% TODO: 학생 구현
    %  (1) lonCmd.Fx_total → 4-wheel 균등 brake (with 60:40 split)
    %  (2) latCmd.yawMoment → 4-wheel 차동 brake
    %  (3) latCmd.steerAngle → actuatorCmd.steerAngle (saturation)
    %  (4) verCmd → actuatorCmd.dampingCoeff (pass-through 또는 추가 가공)
    %  (5) 최종 saturation

    % 1. 초기화 및 차량 파라미터 로드
    Tb = zeros(4, 1);
    rw = 0.33; % 타이어 유효 반경 [m] (추정치)
    tf = VEH.track_f; % 전륜 윤거
    tr = VEH.track_r; % 후륜 윤거
    
    % 2. 종방향 브레이크 기본 분배 (60:40 전후륜 배분)
    if lonCmd.Fx_total < 0
        total_brake_force = abs(lonCmd.Fx_total);
        Fbf = total_brake_force * 0.60;
        Fbr = total_brake_force * 0.40;
        
        Tb(1) = (Fbf / 2) * rw; % FL
        Tb(2) = (Fbf / 2) * rw; % FR
        Tb(3) = (Fbr / 2) * rw; % RL
        Tb(4) = (Fbr / 2) * rw; % RR
    end
    
    % 3. ESC Yaw Moment 차동 브레이크 분배
    Mz = latCmd.yawMoment;
    if abs(Mz) > 0.1
        % 전후륜 비율로 횡방향 모멘트 분담 (60:40)
        delta_F_front = abs(Mz) * 0.6 / (tf / 2);
        delta_F_rear  = abs(Mz) * 0.4 / (tr / 2);
        
        % Mz > 0 (CCW, 반시계 회전 필요) -> 좌측 브레이크(1, 3) 개입
        if Mz > 0 
            Tb(1) = Tb(1) + delta_F_front * rw;
            Tb(3) = Tb(3) + delta_F_rear  * rw;
        % Mz < 0 (CW, 시계 회전 필요) -> 우측 브레이크(2, 4) 개입
        else 
            Tb(2) = Tb(2) + delta_F_front * rw;
            Tb(4) = Tb(4) + delta_F_rear  * rw;
        end
    end
    
    % 4. 물리적 한계점 포화 방지 (Saturation)
    for i = 1:4
        Tb(i) = max(min(Tb(i), LIM.MAX_BRAKE_TRQ), 0);
    end
    
    % 5. Actuator 최종 명령 매핑
    actuatorCmd.steerAngle   = max(min(latCmd.steerAngle, LIM.MAX_STEER_ANGLE), -LIM.MAX_STEER_ANGLE);
    actuatorCmd.brakeTorque  = Tb;
    actuatorCmd.dampingCoeff = verCmd; % 수직 제어기 결과 Pass-through
end