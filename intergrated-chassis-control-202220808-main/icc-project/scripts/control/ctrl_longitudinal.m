function [forceCmd, ctrlState] = ctrl_longitudinal(vxRef, vx, ax, ctrlState, CTRL, LIM, dt)
%CTRL_LONGITUDINAL [학생 작성] 종방향 제어기 (속도 추종 + ABS)
%
%   속도 추종 (cruise/decel) 과 anti-lock braking (slip ratio limiting) 을 통합.
%
%   Inputs:
%       vxRef     - 목표 종방향 속도 [m/s]
%       vx        - 실제 종방향 속도 [m/s]
%       ax        - 종가속도 [m/s²]
%       ctrlState - 내부 상태 (.intError, .prevForce, .wheelSlip(4) 추가 가능)
%       CTRL      - .LON.Kp, .Ki, .intMax
%       LIM       - .MAX_AX, .MAX_JERK, .MAX_BRAKE_TRQ
%       dt        - sample time
%
%   Outputs:
%       forceCmd.Fx_total   - 총 종방향 힘 요구 [N], 양수 가속 / 음수 제동
%       forceCmd.brakeRatio - 제동 비율 (0: 가속, 1: 전제동) — 차후 coordinator 가 brake 토크로 변환
%       ctrlState           - 업데이트
%
%   요구사항:
%       1. 속도 추종 PI 제어
%       2. ABS — wheel slip ratio |κ| > 0.12 일 때 brake force 감소 (slip-limit 또는 bang-bang)
%       3. 저크 제한 (LIM.MAX_JERK · m 으로 force 미분 cap)
%       4. anti-windup
%
%   주의:
%       - 본 함수는 wheel slip 정보가 직접 입력으로 들어오지 않음. 학생은 runner 가 매 step
%         result.tire.{FL,FR,RL,RR}.slipRatio 에 기록하는 값을 ctrlState 에 캐시하는 식으로
%         설계할 수 있음. 또는 ctrl_coordinator 에서 ABS 모듈레이션 (다른 설계 선택).
%       - 본 과제 시나리오 (B1) 는 vxRef 일정 — PID 속도 추종보다 ABS 가 핵심.
%
%   힌트:
%       - slip ratio κ = (ω·r_w - vx) / max(vx, 0.1)
%       - ABS 작동 조건: vehicle 감속 중 (ax < 0) AND |κ| > κ_target (≈0.12)
%       - Bang-bang ABS: brake_cmd = brake_cmd · 0.5 일 때 |κ| > κ_target

    %% TODO: 여기에 학생 구현
    %  (1) speed-tracking PI
    %  (2) ABS modulation (이번 함수에서 또는 ctrl_coordinator 에서)
    %  (3) jerk limit
    %  (4) anti-windup

    %% TODO: 여기에 학생 구현

    % 1. 초기 상태 세팅
    if ~isfield(ctrlState, 'intError')
        ctrlState.intError = 0;
        ctrlState.prevForce = 0;
    end

    mass = 1800; % BMW 5시리즈 예상 질량 [kg] (가정값, 필요시 VEH.mass 참조)

    % 2. 속도 추종 (PI 제어)
    err_v = vxRef - vx;
    Kp = 3000; % [튜닝 포인트]
    Ki = 800;  % [튜닝 포인트]

    ctrlState.intError = ctrlState.intError + err_v * dt;
    % Anti-windup
    ctrlState.intError = max(min(ctrlState.intError, 5000), -5000);

    Fx_raw = (Kp * err_v) + (Ki * ctrlState.intError);

    % 3. 저크(Jerk) 제한 적용 (승차감 및 급격한 하중 이동 방지)
    max_delta_F = LIM.MAX_JERK * mass * dt;
    Fx_cmd = max(min(Fx_raw, ctrlState.prevForce + max_delta_F), ctrlState.prevForce - max_delta_F);

    % 4. 총 가감속력 제한 (MAX_AX 활용)
    max_F = LIM.MAX_AX * mass;
    Fx_cmd = max(min(Fx_cmd, max_F), -max_F);

    ctrlState.prevForce = Fx_cmd;

    % 5. 출력 할당
    forceCmd.Fx_total = Fx_cmd;
    
    if Fx_cmd < 0
        % 제동 시 비율 계산
        forceCmd.brakeRatio = min(abs(Fx_cmd) / max_F, 1.0);
    else
        forceCmd.brakeRatio = 0;
    end
end

