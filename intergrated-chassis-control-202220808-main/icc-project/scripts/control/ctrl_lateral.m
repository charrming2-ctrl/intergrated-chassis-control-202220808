function [deltaAdd, ctrlState] = ctrl_lateral(yawRateRef, yawRate, slipAngle, vx, ctrlState, CTRL, LIM, dt)
%CTRL_LATERAL [학생 작성] 횡방향 통합 제어기 (AFS + ESC)
%
%   yaw rate 추종 (AFS) + slip angle 제한 (ESC) 통합 제어기를 설계하라.
%
%   Inputs:
%       yawRateRef - 목표 yaw rate [rad/s] (driver delta 로부터 bicycle model 로 계산됨)
%       yawRate    - 실제 yaw rate [rad/s]
%       slipAngle  - 차체 슬립 앵글 β [rad]
%       vx         - 종방향 속도 [m/s]
%       ctrlState  - 내부 상태 (.intError, .prevError, ... 자유롭게 확장 가능)
%       CTRL       - sim_params.m 에서 정의된 게인 (.LAT.Kp, .Ki, .Kd, .intMax)
%       LIM        - 한계값 (.MAX_STEER_ANGLE, .MAX_SLIP_ANGLE)
%       dt         - sample time [s]
%
%   Outputs:
%       deltaAdd.steerAngle - AFS 보조 조향각 [rad], 부호 driver delta 와 동일 방향
%       deltaAdd.yawMoment  - ESC 요청 yaw moment [Nm] (ctrl_coordinator 가 brake 차동으로 변환)
%       ctrlState           - 업데이트된 내부 상태
%
%   요구사항:
%       1. yaw rate 추종을 위한 보조 조향 (예: PID, LQR, pole placement, SMC 중 택일)
%       2. |slipAngle| > β_threshold 일 때 yaw moment 인가 (driver intent 와 반대 방향)
%       3. vx 적응 — 저속/고속 게인 differential (예: gain scheduling, LPV)
%       4. anti-windup, saturation 처리
%
%   금지:
%       - scenario id 분기 (예: 'A1 이면 X' 같은 hardcoding)
%       - LIM.MAX_STEER_ANGLE 위반
%       - global 변수 사용
%
%   힌트:
%       - PID 출발점은 sim_params.m 의 CTRL.LAT.Kp/Ki/Kd 값
%       - LQR 설계 시 Bicycle Model state-space (scripts/control/calc_bicycle_model.m 참조)
%       - β-limiter 는 다음 형태가 일반적:
%             if |β| > β_th
%                 M_z = -K_β · sign(β) · (|β| - β_th) · f(vx)
%       - speed scheduling: f(vx) = min(vx/v_ref, 2)

    %% TODO: 여기에 학생 구현 작성
    %  (1) PID/LQR/... 으로 yaw rate 추종 보조 조향 계산
    %  (2) slip angle 임계 초과 시 yaw moment 계산
    %  (3) speed scheduling 적용
    %  (4) limit/saturation

    %% TODO: 여기에 학생 구현 작성
    
    % 1. 초기 상태 세팅 (적분기 및 이전 오차)
    if ~isfield(ctrlState, 'intError')
        ctrlState.intError = 0;
        ctrlState.prevError = 0;
    end

    % 2. AFS (Active Front Steer) - Yaw Rate 추종 (PID 제어)
    error_yr = yawRateRef - yawRate;
    
    % 속도 감응형 게인 스케줄링 (고속에서 조향 민감도 하향)
    % 명세서 힌트 적용: f(vx) = min(vx/v_ref, 2)
    v_ref = 20.0; 
    speed_factor = max(0.5, min(v_ref / max(vx, 1.0), 2.0));
    
    % [튜닝 포인트] 기본 게인은 sim_params.m의 CTRL.LAT 파라미터로 교체 권장
    Kp = 0.8 * speed_factor; 
    Ki = 0.1 * speed_factor;
    Kd = 0.05 * speed_factor;

    % Anti-windup이 적용된 적분기
    ctrlState.intError = ctrlState.intError + error_yr * dt;
    intMax = 0.5; % 적분기 한계
    ctrlState.intError = max(min(ctrlState.intError, intMax), -intMax);

    derivError = (error_yr - ctrlState.prevError) / dt;
    ctrlState.prevError = error_yr;

    % 보조 조향각 계산 및 포화(Saturation) 방지
    delta_afs = (Kp * error_yr) + (Ki * ctrlState.intError) + (Kd * derivError);
    delta_afs = max(min(delta_afs, LIM.MAX_STEER_ANGLE), -LIM.MAX_STEER_ANGLE);

    % 3. ESC (Electronic Stability Control) - Slip Angle 제한
    % 명세서 기준 가장 엄격한 A4/A1 시나리오 대응 (각각 2도, 3도)
    beta_th = 2.5 * (pi / 180); % 2.5도를 임계점으로 설정 (rad 변환)
    M_z = 0;
    
    if abs(slipAngle) > beta_th
        % 슬립 각도 폭주 방지를 위한 강한 복원 모멘트 생성
        K_beta = 60000; % [튜닝 포인트] 모멘트 게인
        M_z = -K_beta * sign(slipAngle) * (abs(slipAngle) - beta_th) * max(vx / 10, 1);
    end

    % 4. 출력 할당
    deltaAdd.steerAngle = delta_afs;
    deltaAdd.yawMoment  = M_z;
end
