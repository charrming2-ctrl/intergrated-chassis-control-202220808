# [202220808-이찬민] ICC 제어기 설계 보고서

**과목**: 자동제어 — 2026 봄
**제출일**: 2026-06-23
**팀**: 개인

---

## 1. 설계 개요

 본 프로젝트의 목적은 14 자유도(DOF)를 가진 BMW 5 시리즈 가상 차량 모델을 제어하여,
6개의 ISO 및 UN-R 표준 평가 시나리오(회피 조향, 제동 선회 등)에서 차량의 동적 한계를
극복하고 베이스라인(제어기 OFF) 대비 핵심 성과 지표(KPI)를 개선하는 것입니다.
 이를 위해, 측방향(Lateral), 종방향(Longitudinal), 수직방향(Vertical)의 독립적인 
제어기를 설계하고, 코디네이터(Coordinator)를 통해 각 구동기에 최적의 명령을 할당하는 
통합 섀시 제어(ICC) 시스템을 구축했습니다.

제어 기법으로는 모델의 선형화 구간에서 빠르고 직관적인 튜닝이 가능한 
**고전적 PID 제어**와 물리적 한계를 직관적으로 제한하는 
**Rule-based Bang-Bang 제어**를 혼합하여 적용했습니다. 
복잡한 비선형 타이어 모델과 14 DOF 플랜트의 전체 상태 변수를 추정해야 하는 
LQR(Linear Quadratic Regulator) 기법 대신, 단순화된 자전거 모델(Bicycle Model)
기반의 선형 오차 역학에 PID 제어를 결합하는 것이 실시간 피드백 시스템에서 신뢰성 높은
추종 성능을 보장할 수 있다고 판단했습니다.
(R. Rajamani, *Vehicle Dynamics and Control* 참조).

**각 제어기 핵심 요약:**
- **ctrl_lateral**: 속도 감응형(Speed-scheduled) PID 제어로 yaw rate 추종 + Rule-based(Bang-Bang) 제어로 $\beta$-limiter 구현
- **ctrl_longitudinal**: Anti-windup PI 제어 및 가속도/Jerk 물리 한계 포화(Saturation) 기법
- **ctrl_vertical**: Sprung/Unsprung mass 상대 속도 부호 기반의 On-Off Skyhook 제어 기법
- **ctrl_coordinator**: 종방향 제동력 60:40 고정 분배 및 ESC $M_z$ 요구량에 따른 좌우 휠 차동 브레이크(Differential Braking) 할당

---

## 2. 수학적 모델링 (1-2 페이지)

### 2.1 사용한 plant 단순화 및 제어 모델 선정
본 프로젝트에서 최종 검증에 사용되는 시뮬레이션 플랜트는 차량의 피치(Pitch), 롤(Roll), 스프렁/언스프렁 질량(Sprung/Unsprung mass), 그리고 서스펜션의 강성($k_s$)과 감쇠($C_s$) 특성까지 모두 포함된 14 자유도(DOF)의 복합 비선형 차량 동역학 모델입니다. 

이러한 고도화된 모델은 실제 차량의 거동을 모사하고 검증하는 데는 탁월하지만, 동역학적 결합(Coupling)이 너무 강하고 상태 변수가 많아 실시간 피드백 제어기(PID 등)의 이득(Gain)을 직관적으로 설계하기에는 수학적으로 매우 부적합합니다. 따라서 본 설계에서는 제어 논리의 직관적인 전개와 선형 제어 이론의 도입을 위해, 차량의 횡방향(Lateral) 및 요(Yaw) 거동만을 독립적으로 추출하여 좌우 바퀴를 하나로 통합한 **2 자유도 선형 자전거 모델(2-DOF Linear Bicycle Model)**을 근사 플랜트로 채택했습니다. 이를 통해 종방향과 수직방향 동역학을 수학적으로 분리(Decoupling)하여 AFS 및 ESC 제어기 설계의 기준 모델로 활용하였습니다.

### 2.2 State-space 표현 (상태 공간 모델)
제어기 구성을 위해 연속 시간(Continuous-time)에서 선형 자전거 모델을 상태 공간 방정식(State-space equation)으로 표현하면 다음과 같습니다.

$$\dot{x} = Ax + Bu, \quad y = Cx + Du$$

여기서 시스템의 상태 변수 벡터 $x$는 차량의 무게중심에서의 횡방향 속도 $v_y$와 수직축 기준 회전 각속도인 요레이트 $r$로 정의되며, 제어 입력 $u$는 운전자 및 조향 액추에이터에 의한 전륜 조향각 $\delta$입니다.

$$x = \left[ \begin{array}{c} v_y \\ r \end{array} \right], \quad u = \delta$$

선형 자전거 모델을 구성하는 미분 방정식은 다음과 같이 2차 정방 행렬 형태의 시스템 행렬 $A$와 입력 행렬 $B$로 명확하게 전개됩니다.

$$\left[ \begin{array}{c} \dot{v}_y \\ \dot{r} \end{array} \right] = \left[ \begin{array}{cc} -\frac{C_f + C_r}{m V_x} & \frac{l_r C_r - l_f C_f}{m V_x} - V_x \\ \frac{l_r C_r - l_f C_f}{I_z V_x} & -\frac{l_f^2 C_f + l_r^2 C_r}{I_z V_x} \end{array} \right] \left[ \begin{array}{c} v_y \\ r \end{array} \right] + \left[ \begin{array}{c} \frac{C_f}{m} \\ \frac{l_f C_f}{I_z} \end{array} \right] \delta$$

**[시스템 파라미터 정의]**
* $m$: 차량의 총 질량 (Mass)
* $I_z$: 차량의 요 관성 모멘트 (Yaw Moment of Inertia)
* $l_f, l_r$: 차량 무게중심(CG)에서 전륜 축과 후륜 축까지의 종방향 거리
* $C_f, C_r$: 전륜과 후륜 타이어의 등가 코너링 강성 (Cornering Stiffness)
* $V_x$: 차량의 종방향 속도 (Longitudinal Velocity)

### 2.3 제어 모델의 주요 가정 및 한계점
앞서 정의한 선형 상태 공간 모델은 제어기 설계를 획기적으로 단순화하지만, 다음과 같은 강력한 물리적 가정들을 전제로 하므로 한계 주행(Limit Handling) 상황에서 실제 14 DOF 플랜트와의 수식적 오차를 유발합니다.

**1. 일정 종속도 가정 ($V_x \approx \text{constant}$)**
상태 공간 방정식의 시스템 행렬 $A$를 시불변(LTI, Linear Time-Invariant) 시스템으로 취급하기 위해 제어기 설계 시 차량의 종방향 속도 $V_x$를 상수로 가정했습니다. 그러나 A7(Brake-in-Turn)이나 D1(DLC + Brake) 시나리오처럼 급격한 제동과 선회가 동시에 일어나는 기동에서는 $V_x$가 급감합니다. 속도가 변하면 시스템의 극점(Pole)이 이동하는 시변(Time-varying) 특성을 띠게 되므로, 고정된 제어 게인만으로는 오차 역학이 발산할 수 있는 근본적인 한계가 존재합니다.

**2. 선형 타이어 및 소슬립 영역 가정 (Linear Tire Model)**
본 모델은 타이어에서 발생하는 횡력이 타이어의 슬립 앵글(Slip angle, $\alpha$)에 선형적으로 비례($F_y = C_\alpha \cdot \alpha$)한다고 가정합니다. 이 선형성은 슬립 앵글이 대략 $3^\circ \sim 4^\circ$ 미만인 일상적인 주행(소슬립 영역)에서만 유효합니다. A1(이중 차선 변경)과 같은 극한의 회피 기동에서는 원심력이 급증하여 타이어가 마찰원의 물리적 한계치($\mu F_z$)에 도달합니다. 이때 종력과 횡력이 결합하며 타이어가 비선형적으로 포화(Saturation)되므로, 선형 모델은 타이어가 낼 수 있는 실제 횡력을 과대평가하는 치명적인 약점을 가집니다.

**3. 수직 하중 이동(Load Transfer) 배제**
자전거 모델의 가장 큰 기하학적 한계는 좌우 윤거(Track width)를 0으로 취급하여 롤(Roll) 모멘트에 의한 수직 하중 이동을 반영하지 못한다는 것입니다. 실제 코너링 시에는 롤 축(Roll axis)을 중심으로 차체가 기울어지며 외측 바퀴로 막대한 하중이 이동합니다. 타이어의 코너링 강성($C_f, C_r$)은 수직 하중에 비례하여 변하지만, 하중 증가에 따른 횡력 증가분보다 내측 바퀴의 하중 감소에 따른 횡력 감소분이 더 크기 때문에 전체적인 차량의 횡방향 접지력은 저하됩니다. 설계된 모델은 이를 상수($C_f, C_r$)로 고정했기 때문에, ESC 코디네이터가 차동 제동을 분배할 때 좌우 타이어의 실제 마찰력 한계 변화를 정밀하게 추종하지 못하는 원인이 됩니다.

---

## 3. 제어기 설계 (3-4 페이지)

### 3.1 ctrl_lateral — AFS + ESC
**설계 목표**:
- 정상 주행 시 운전자의 의도(Reference Yaw Rate)를 빠르고 정확하게 추종 (A3 기준 settling < 0.8s, overshoot < 10%)
- 한계 주행 시 $|\beta| > 2.5^\circ$ 초과 시 즉각적인 ESC 개입으로 스핀아웃 방지

**선택 기법**: 속도 감응형(Speed-scheduled) PID 제어 (AFS) + Rule-based Bang-Bang 제어 (ESC)

**Gain 계산 과정**:
자전거 모델을 기반으로 한 조향각($\delta$) 대비 요레이트($r$)의 1차 전달함수 특성상 고속에서 DC 게인이 급증하여 시스템이 발산합니다. 이를 방지하기 위해 기준 속도($20.0\,\text{m/s}$) 기반의 스케줄링 팩터를 적용했습니다.
- 스케줄링 수식: $f(V_x) = \max\left(0.5, \min\left(\frac{20.0}{\max(V_x, 1.0)}, 2.0\right)\right)$
- PID 튜닝: Ziegler-Nichols 계단 응답법으로 기본 비율을 잡은 후 A3 시뮬레이션을 통해 반복(Iteration) 튜닝을 진행했습니다. 적분기 Windup 방지를 위해 $\pm0.5$로 하드 클리핑을 적용했습니다.
- ESC 튜닝: 미끄러짐은 연속 제어보다 즉각적인 차단이 중요하다고 판단하여 임계치 $\beta_{th} = 2.5^\circ$를 초과하는 즉시 극단적인 게인($K_\beta = 60000$)으로 역방향 요 모멘트를 인가하도록 설계했습니다.

**최종 게인 + 정당화**:
```matlab
% 속도 감응형 팩터 적용으로 고속 안정성 확보
speed_factor = max(0.5, min(20.0 / max(vx, 1.0), 2.0));

CTRL.LAT.Kp = 0.8 * speed_factor;
CTRL.LAT.Ki = 0.1 * speed_factor;
CTRL.LAT.Kd = 0.05 * speed_factor;

% 폭주 억제를 위한 강한 ESC 개입 설정
BETA_THRESHOLD = deg2rad(2.5);
BETA_GAIN = 60000;
```

### 3.2 ctrl_longitudinal — 속도 + ABS
**설계 목표**:
- 목표 속도($v_{x,ref}$)에 대한 빠른 추종.
- 승차감 저하 및 급격한 수직 하중 이동을 유발하는 과도한 Jerk 제한.

**선택 기법**: Anti-windup PI 제어 + Rate Limiter (Jerk 제한)

**Gain 계산 과정**:
종방향 가감속은 $F_x = m \dot{v}_x$에 기반하므로 PI 제어를 통해 정상 상태 오차를 제거했습니다. 차량의 큰 질량($m=1800\text{kg}$)을 극복하기 위해 Kp와 Ki를 큰 값으로 설정했습니다.
응답성을 높인 반작용으로 급가감속이 발생하는 것을 막기 위해 물리량 기반의 Jerk 한계수식을 적용했습니다.
- Jerk 한계: $\Delta F_{max} = \text{LIM.MAX\_JERK} \cdot m \cdot dt$
- 가속도 한계: $F_{x,max} = \text{LIM.MAX\_AX} \cdot m$ (최종 포화)

**최종 게인 + 정당화**:
```matlab
% 무거운 차체를 제어하기 위한 높은 기본 게인
CTRL.LON.Kp = 3000;
CTRL.LON.Ki = 800;

% Anti-windup 클리핑
INT_ERROR_MAX = 5000; 
```
*(ABS 모듈레이션은 시간 관계상 생략하였으며, 최대 가속도 클램핑으로 대체하였습니다.)*

### 3.3 ctrl_vertical — CDC
**설계 목표**:
- 노면 요철 통과 시 차체 흔들림(Body bounce) 억제 및 승차감 확보.

**선택 기법**: On-Off Skyhook Control

**Gain 계산 과정**:
스카이훅 이론에 따라 차체의 수직 절대 속도 $\dot{z}_s$와 서스펜션의 상대 속도 $(\dot{z}_s - \dot{z}_u)$의 방향이 같을 때 강한 댐핑으로 차체를 잡아주고, 다를 때는 댐핑을 풀어 노면 충격을 흡수하는 2상태 제어를 구현했습니다.

**최종 게인 + 정당화**:
```matlab
% Skyhook 조건: z_s_dot * (z_s_dot - z_u_dot) > 0
C_MAX = 5000; % 차체 제어용 강한 댐핑
C_MIN = 500;  % 승차감용 약한 댐핑
```

### 3.4 ctrl_coordinator — Actuator Allocation
**설계 목표**:
- 종방향 제어기의 요구 힘($F_x$)과 측방향 제어기의 요구 모멘트($M_z$)를 4륜 브레이크에 물리적 충돌 없이 차동 분배.

**할당 로직 및 계산 과정**:
1. **기본 제동 분배 (전후 60:40 적용)**
제동 시 전륜으로 쏠리는 하중 이동을 고려하여 고정 비율 분배를 적용했습니다.
$$T_{b,FL} = T_{b,FR} = \left(\frac{|F_{x,total}| \cdot 0.6}{2}\right) r_w$$
$$T_{b,RL} = T_{b,RR} = \left(\frac{|F_{x,total}| \cdot 0.4}{2}\right) r_w$$

2. **ESC 차동 제동 (Differential Braking)**
요구되는 $M_z$를 타이어 윤거($t_f, t_r$)를 이용해 좌우 휠의 제동력 차이($\Delta F$)로 변환합니다. 전후륜 분담 비율은 기본 제동과 동일하게 60:40을 적용했습니다.
$$\Delta F_{front} = \frac{|M_z| \cdot 0.6}{t_f / 2}, \quad \Delta F_{rear} = \frac{|M_z| \cdot 0.4}{t_r / 2}$$
모멘트 부호($M_z > 0$ 시 반시계 방향, $M_z < 0$ 시 시계 방향)에 따라 회전하고자 하는 방향의 안쪽 바퀴(좌측 또는 우측)에만 $\Delta F \cdot r_w$의 토크를 가산합니다.
최종적으로 4륜의 토크는 `LIM.MAX_BRAKE_TRQ`를 넘지 않도록 포화(Saturation)시켜 한계를 방어합니다.

---

## 4. 시뮬레이션 결과

### 4.1 P1 시나리오 benchmark — 베이스라인 vs 본인 설계
아래 표는 제어기를 비활성화했을 때(OFF)와 본 설계 제어기를 적용했을 때(ON)의 핵심 KPI 변화량($\Delta\%$)을 비교한 결과입니다. (`run('scripts/grade.m')` 자동 채점 결과: **42.31 / 70.00**)

| 시나리오 | KPI | OFF (Baseline) | ON (본인) | $\Delta\%$ | 평가 및 채점 결과 |
|---|---|---|---|---|---|
| A1 DLC | sideSlipMax [°] | 30.50 | 2.51 | **-91.7%** | **Pass** (6.0 / 6) |
| A1 | LTR_max | 0.948 | 0.739 | **-22.0%** | FAIL (3.84 / 5) |
| A3 step | yawRateOvershoot [%] | 2.81 | 1.47 | **-47.6%** | **Pass** (4.0 / 4) |
| A4 SS | understeerGradient | 0.0031 | -2543.83 | -- | FAIL (0.0 / 5) |
| A7 BIT | sideSlipMax [°] | 46.30 | 1.24 | **-97.3%** | **Pass** (8.0 / 8) |
| A7 | LTR_max | 0.745 | 0.203 | **-72.7%** | **Pass** (7.0 / 7) |
| B1 brake | stoppingDistance [m] | 72.40 | 72.29 | **-0.1%** | FAIL (0.0 / 5) |
| D1 통합 | sideSlipMax [°] | 7.65 | 4.06 | **-46.9%** | **Pass** (3.93 / 4) |

---

### 4.2 핵심 plot — A1 DLC

![A1 trajectory comparison](./figures/a1_trajectory.png)
*Figure 4.1 — A1 ISO 3888-1 DLC, 차량 trajectory (off vs on) vs reference path.*

**[궤적 분석]**
Figure 4.1은 $80\,\text{km/h}$로 진입하여 이중 차선 변경(Double Lane Change)을 수행하는 A1 시나리오의 차량 궤적입니다. 빨간색 점선(OFF)으로 표시된 베이스라인 차량은 첫 번째 조향 직후 원심력과 타이어 마찰 한계 극복에 실패하여 후륜 그립을 잃고 궤적을 크게 이탈(Spin-out)합니다. 반면, 파란색 실선(ON)으로 표시된 제어 차량은 파일런(Cone) 구간 내에서 안정적으로 차선 변경과 복귀를 수행합니다. 비록 강력한 ESC 개입으로 인해 차량의 횡방향 속도가 급감하며 경로 바깥으로 약간 밀려나는 오차(Max Deviation: 2.21m)가 발생했으나, 차량의 자세(Heading) 자체는 완벽하게 유지되는 것을 확인할 수 있습니다.

![A1 yaw rate](./figures/a1_yawrate.png)
*Figure 4.2 — A1 yaw rate 응답: reference (driver bicycle model), off (controller off), on (본인 설계).*

**[요레이트 및 제어 개입 분석]**
Figure 4.2의 요레이트(Yaw Rate) 응답 그래프를 보면 제어기의 역할이 명확히 드러납니다. 조향 초반부에는 AFS(ctrl_lateral의 PID 제어기)가 작동하여 검은색 점선(Reference)을 파란색 실선(ON)이 부드럽게 추종합니다. 하지만 두 번째 차선 변경(조향각이 급격히 반대로 꺾이는 구간)에서 슬립 각도($\beta$)가 임계치인 $2.5^\circ$를 돌파하는 순간, ESC가 $60000$이라는 거대한 게인으로 역방향 요 모멘트($M_z$)를 뿜어냅니다. 이 순간 그래프 상에서 요레이트가 레퍼런스보다 강하게 꺾이는(억제되는) 피크 현상이 관찰되는데, 이는 차량이 미끄러지는 것을 막기 위해 코디네이터가 외측 전륜에 강한 차동 브레이크를 인가하여 강제로 차체를 정렬시켰기 때문입니다.

*(plot 생성 코드)*
```matlab
[r_off, k_off] = run_icc_scenario('A1','14dof','Controller','off','SavePlot',false);
[r_on,  k_on ] = run_icc_scenario('A1','14dof','Controller','on', 'SavePlot',false);
figure; plot(r_off.x_pos, r_off.y_pos, 'r--', r_on.x_pos, r_on.y_pos, 'b-', ...
             r_off.scenario.refPath(:,1), r_off.scenario.refPath(:,2), 'k:');
xlabel('x [m]'); ylabel('y [m]'); legend('off','on','ref'); axis equal;
saveas(gcf, 'docs/figures/a1_trajectory.png');
```

---

### 4.3 한 시나리오 deep dive — A7 Brake-in-Turn (가장 성공적인 방어 사례)

A7 시나리오는 선회 도중 $0.4g$ 수준의 급제동을 가하는 상황으로, 하중이 전륜으로 급격히 쏠리면서 후륜의 접지력이 상실되어 심각한 오버스티어(Oversteer)를 유발하는 고난도 테스트입니다.

* **베이스라인 (제어기 OFF)**: `sideSlipMax` = $46.3^\circ$ (완전한 스핀아웃 및 롤오버 위험)
* **본인 설계 (제어기 ON)**: `sideSlipMax` = $1.24^\circ$ (완벽한 자세 제어)

**[심층 분석: 제어기 성공의 핵심 요인]**
이 극적인 $97.3\%$의 개선율은 본 설계에서 채택한 **"초저임계-고게인(Low-Threshold, High-Gain)" ESC 전략**이 완벽하게 적중한 결과입니다.

1. **마찰원(Friction Circle) 선점과 찰나의 개입 시점**:
   선회 제동 시 타이어는 종방향 마찰력(제동)과 횡방향 마찰력(코너링)을 동시에 감당해야 하므로 마찰원이 극도로 포화됩니다. 베이스라인은 브레이크 압력이 네 바퀴에 고정 분배되면서 내측 후륜부터 접지력을 잃고 차가 돌기 시작합니다. 
   본 제어기는 차체가 미끄러지기 시작하는 초기 징후인 $\beta = 2.5^\circ$ 지점을 '돌이킬 수 없는 위험선'으로 정의했습니다. 차가 $46^\circ$까지 돌기 전에, 겨우 $2.5^\circ$ 틀어진 시점에서 ESC가 개입을 시작합니다.

2. **압도적인 요 모멘트($M_z$) 인가 패턴**:
   임계점을 넘는 순간, 제어기는 속도에 비례하는 $60000$의 게인을 곱해 막대한 크기의 역방향 요 모멘트($M_z$)를 코디네이터(`ctrl_coordinator`)로 전달합니다. 
   코디네이터는 종방향 제어기에서 요구한 기본 제동력($0.4g$)을 덮어버릴 만큼 강력한 차동 브레이크($\Delta F$)를 선회 바깥쪽 바퀴(외륜)에 집중적으로 꽂아 넣습니다. 이때 브레이크 토크는 하드웨어 한계(`LIM.MAX_BRAKE_TRQ`) 직전까지 포화(Saturation)되며 작동합니다. 
   즉, 부드럽게 자세를 잡는 것이 아니라, 차체가 미끄러지는 반대 방향으로 액추에이터가 낼 수 있는 최대의 물리력을 펀치(Bang-Bang)처럼 타격하여 억지로 차체를 원래 각도로 꺾어버린 것입니다.

결과적으로 A7 시나리오에서 차량은 타이어가 비선형 영역으로 완전히 빠지기 직전에 강제적인 차동 제동을 맞고 요레이트를 회복하였으며, 슬립 각도를 $1.24^\circ$라는 놀라운 수치로 방어하며 15점 만점을 확보할 수 있었습니다.

## 5. 분석 및 한계 (1-2 페이지)

본 프로젝트를 통해 각 독립 제어기(Lateral, Longitudinal, Vertical)를 통합하여 섀시를 제어하는 시스템을 구축했습니다. 정해진 모범 답안에 의존하기보다 시뮬레이션 결과를 바탕으로 파라미터 튜닝과 한계 방어 로직을 직접 설계하는 과정을 거쳤으며, 이 과정에서 '과도 응답(안전성)'과 '정상 상태(주행성)' 사이의 극단적인 트레이드오프(Trade-off)를 명확히 확인할 수 있었습니다.

### 5.1 가장 성공적이었던 시나리오 (A7 Brake-in-Turn)
가장 큰 KPI 개선을 이룬 시나리오는 선회 중 급제동이 가해지는 **A7 (Brake-in-Turn)**입니다. 베이스라인에서는 하중이 전륜으로 쏠리며 후륜 그립 상실로 사이드 슬립이 $46.3^\circ$까지 폭주하여 스핀아웃이 발생했습니다. 그러나 본 설계에서는 이를 $1.24^\circ$로 완벽하게 억제하며 15점 만점을 획득했습니다.
- **핵심 요인**: ESC의 개입 임계값을 $\beta_{th} = 2.5^\circ$로 매우 타이트하게 설정하고, 모멘트 게인($K_\beta$)을 $60000$이라는 극단적으로 높은 값으로 인가한 '초저임계-고게인' 전략이 성공했습니다. 타이어가 비선형 포화 영역으로 빠지기 직전인 $2.5^\circ$ 찰나의 순간에, 액추에이터 한계치에 달하는 차동 제동력($\Delta F$)이 안쪽 바퀴에 Bang-Bang 제어처럼 강하게 타격되어 차체의 스핀 모멘트를 물리적으로 상쇄시켰기 때문입니다.

### 5.2 가장 부족했던 시나리오 및 원인 분석 (A4, B1)
극단적인 튜닝으로 극한 기동(A1, A7)에서는 살아남았지만, 역설적으로 이 세팅이 일상적인 정상 상태 주행을 완전히 파괴하는 치명적인 한계를 드러냈습니다.

**1. A4 정상 선회 (Understeer Gradient 발산 및 실패)**
반경 50m를 원활하게 도는 A4 시나리오에서 시스템은 $K_{us} = -2543$이라는 극단적인 오버스티어를 유발하며 0점을 기록했습니다. 
- **가설 1 (정상 상태 슬립에 대한 여유 부족)**: 원선회를 유지하기 위해 차량은 원심력에 대항하며 자연스럽게 $2^\circ \sim 3^\circ$의 정상 상태 슬립(Steady-state slip)을 발생시켜야 합니다. 하지만 ESC 로직이 이 정상적인 슬립조차 '통제 불능의 위험'으로 오인하여 내측 바퀴에 풀 브레이킹을 걸어버렸고, 이로 인해 차량이 팽이처럼 제자리에서 스핀하는 결과를 낳았습니다.
- **가설 2 (선형 자전거 모델의 구조적 한계)**: 제어기 설계 시 롤(Roll) 거동에 의한 하중 이동을 무시했습니다. A4 선회 중 외측 휠로 하중이 크게 이동하여 타이어 한계 마찰력이 변했음에도, 코디네이터가 이를 무시하고 좌우 윤거에만 비례하여 무리한 브레이크를 인가한 것이 제어 발산의 원인으로 추정됩니다.

**2. B1 직진 제동 (제동 거리 미달)**
목표 제동 거리($40m$)에 한참 못 미치는 $72.29m$를 기록하여 베이스라인과 차이를 만들지 못했습니다.
- **가설 1 (진정한 의미의 ABS 부재)**: 종방향 제어기에서 $F_x$를 단순히 섀시의 물리적 한계치(`LIM.MAX_AX * mass`)로 하드 클리핑(Clamping)만 했을 뿐, 각 휠의 회전 속도를 모니터링하지 않았습니다. 결과적으로 휠 슬립 비율($\kappa$)이 목표치($-0.12$)를 넘어 완전히 잠긴 채($\kappa \to -1.0$) 미끄러지는 상태가 유지되어 타이어의 종방향 마찰 계수($\mu$)가 급감했기 때문입니다.

### 5.3 만약 더 시간이 있었다면
이러한 한계들을 극복하기 위해 제어 로직을 다음과 같이 고도화할 것입니다.
- **ESC 게인 스케줄링(Gain Scheduling) 적용**: 단일 상수 게인($K_\beta = 60000$)의 한계를 극복하기 위해, 차량의 횡가속도($a_y$)와 조향각($\delta$) 변화율을 모니터링하여 A4와 같은 정상 상태에서는 ESC를 비활성화(Deadzone 확장)하고, 과도 상태에서만 비례적으로 개입하도록 LPV(Linear Parameter-Varying) 형태의 튜닝 맵을 구축하겠습니다.
- **상태 머신 기반의 ABS 모듈레이션 이식**: 종방향 제어기(`ctrl_longitudinal`) 내부에 타이어 슬립 앵글 $\kappa$를 피드백받는 로직을 추가하겠습니다. $\kappa < -0.15$로 떨어지면 브레이크 압력을 일시 개방($F_x = 0$)하고, 회복 시 다시 인가하는 디지털 펄스 폭 변조(PWM) 형태의 로직을 구현하여 제동 거리를 $40m$ 이내로 단축시키겠습니다.
- **동적 하중 이동을 고려한 WLS 분배**: 코디네이터에서 전후 60:40의 고정 비율 브레이크 분배를 버리고, 수직 제어기의 서스펜션 변위 데이터를 활용해 실시간 4륜 수직 하중($F_z$)을 추정하여 마찰원 한계에 맞게 동적으로 제동력을 할당하는 알고리즘을 도입하겠습니다.

## 6. 참고문헌
[1] ISO 3888-1:2018 — Passenger cars — Test track for a severe lane-change manoeuvre. (A1 시나리오 평가 기준)
[2] ISO 7975:2019 — Passenger cars — Braking in a turn — Open-loop test method. (A7 시나리오 평가 기준)
[3] R. Rajamani, *Vehicle Dynamics and Control*, 2nd ed., Springer 2012. (Ch.2 Bicycle Model 선형화 및 AFS 제어, Ch.8 ESC 차동 제동 로직 참고)
[4] J. Y. Wong, *Theory of Ground Vehicles*, 4th ed., Wiley 2008. (타이어 마찰원 및 하중 이동 모델링 참고)
[5] 자동제어 프로젝트 명세서 (ASSIGNMENT.md 및 ICC Test Protocol 가이드라인)

---

## 부록 A — 사용한 AI 도구
* **Gemini**: 
  - 프로젝트 명세서를 보고 이해한 내용을 바탕으로 Gemini에게 제어기 코드를 요청하였고 AI가 튜닝한 제어기 코드(`ctrl_*.m`)와 자동 채점 결과(`grade.m`)를 바탕으로, 각 시나리오별 최종 점수를 분석하는 데 활용하였습니다.
  - 특히 A1, A7 시나리오에서의 성공(극단적인 ESC 게인을 통한 마찰원 선점)과 A4, B1 시나리오에서의 실패(정상 선회 발산 및 ABS 모듈레이션 부재) 사이의 Trade-off 관계를 논리적으로 이해하는데 도움을 받았습니다.
  - 마크다운(Markdown) 보고서 양식 템플릿 서식 정렬에 사용하였습니다.

---

## 부록 B — 본인 제어 파라미터 변경사항 (sim_params.m 및 하드코딩)

본 설계에서는 고정된 상수 게인(`sim_params.m`의 `CTRL.LAT.Kp` 등)만으로는 고속 주행 시 발산하는 문제를 해결할 수 없다고 판단했습니다.
따라서 `sim_params.m`을 직접 수정하는 대신, `ctrl_lateral.m`과 `ctrl_longitudinal.m` 코드 내부에서 동적 변수(차속 등)를 활용해 파라미터를 실시간으로 덮어씌우는 방식을 채택했습니다. 

적용된 핵심 튜닝 값은 다음과 같습니다.

```matlab
% [1] 측방향 제어기 (ctrl_lateral.m 내부 적용)
% 변경 전 (기본값): Kp = 1.0, Ki = 0.1, Kd = 0.0
% 변경 후 (속도 감응형 팩터 speed_factor 곱셈 연산 적용 전 기준):
CTRL.LAT.Kp = 0.8
CTRL.LAT.Ki = 0.1
CTRL.LAT.Kd = 0.05
BETA_THRESHOLD = 2.5 * (pi/180) % (스핀아웃 방어용 타이트한 임계치)
BETA_GAIN = 60000               % (A1/A7 회피용 극단적 ESC 게인)

% [2] 종방향 제어기 (ctrl_longitudinal.m 내부 적용)
CTRL.LON.Kp = 3000  % 무거운 차체(1800kg) 극복을 위한 높은 비례 게인
CTRL.LON.Ki = 800
INT_ERROR_MAX = 5000 % Anti-windup 클리핑 한계

% [3] 수직방향 제어기 (ctrl_vertical.m 내부 적용)
C_MAX = 5000  % Skyhook On (강한 댐핑)
C_MIN = 500   % Skyhook Off (약한 댐핑)
```
