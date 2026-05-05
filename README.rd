# Quadcopter Delivery Drone with Adaptive PID Control Under Payload Variation

**16-299 Final Project**  
Riya Kadakia, Xinwan Wang, Nidhi Vadlamudi  
May 2026

---

## Abstract

This project investigates the stability and performance of a 
quadrotor delivery drone under abrupt payload variation using 
cascaded PID control with adaptive gain scheduling. We implement 
the full nonlinear quadrotor dynamics in MATLAB/Simulink using 
Newton-Euler equations of motion, and evaluate a fixed-gain PID 
baseline against an adaptive gain-scheduled PID controller across 
three scenarios: no payload, payload pickup (0.5→0.8 kg), and 
payload drop-off. Our adaptive controller reduces peak altitude 
deviation from ~2.3m to ~1.3m, eliminates steady-state error, 
and maintains stability under air disturbances and asymmetric 
drop torques — demonstrating that gain scheduling is a viable 
and practical approach for payload-aware drone control.

---

## Introduction

Real-world delivery drones face a fundamental control challenge: 
they must remain stable despite significant and abrupt changes 
in payload mass. A drone picking up a package may increase its 
total mass by 60% or more, fundamentally altering its dynamics. 
Standard PID controllers tuned for a fixed operating point 
struggle with these sudden parameter changes, producing overshoot, 
oscillations, or temporary instability.

Most classroom examples assume constant mass. This project extends 
the standard cascaded PID framework to a more realistic scenario 
by introducing time-varying mass parameters and testing controller 
robustness in simulation. We ask: can adaptive gain scheduling — 
where PID gains update in real-time based on known payload state — 
meaningfully improve flight stability during pickup and drop-off 
events?

We base our dynamics model on Azizi et al. (2022) and Sahrir & 
Basri (2018), using Newton-Euler equations of motion validated 
against published settling times and overshoot figures.

---

## Background

Prior work has established several approaches to quadrotor control 
under varying conditions. Azizi et al. (2022) demonstrated 
baseline PID performance for hover and trajectory tracking using 
manually-tuned cascaded PID in MATLAB/Simulink. Argentim et al. 
(2013) compared PID, LQR, and a hybrid LQR-PID approach, finding 
LQR provided more stable output with no overshoot at the cost of 
greater mathematical complexity. MathWorks provides a nonlinear 
MPC example showing how predictive control handles motor 
saturation across 12 states and 4 inputs.

Our work sits between these approaches — more adaptive than fixed 
PID but simpler to implement than LQR or MPC — making it 
practical for deployment on resource-constrained platforms like 
the Bitcraze Crazyflie 2.1+.

---

## Methodology

### Quadrotor Dynamics Model

We implemented the Newton-Euler equations of motion in a MATLAB 
Function block within Simulink, tracking 8 states:

- Angular positions: roll (φ), pitch (θ), yaw (ψ)
- Angular rates: φ̇, θ̇, ψ̇  
- Altitude: z
- Vertical velocity: ż

The equations of motion are:

φ̈ = (1/Jx)[(Jy−Jz)θ̇ψ̇ + l·U2]  
θ̈ = (1/Jy)[(Jz−Jx)φ̇ψ̇ + l·U3]  
ψ̈ = (1/Jz)[(Jx−Jy)φ̇θ̇ + U4]  
z̈ = (1/m)[cos(φ)cos(θ)·U1] − g  

Physical parameters used:
| Parameter | Value |
|-----------|-------|
| Mass (m) | 0.5 kg |
| Arm length (l) | 0.2 m |
| Jx = Jy | 4.85×10⁻³ kg·m² |
| Jz | 8.81×10⁻³ kg·m² |
| Jr | 3.36×10⁻⁵ kg·m² |

**Open-loop validation:** Applying U1 = m·g held altitude 
steady. Increasing U1 caused climb, decreasing caused descent — 
confirming correct dynamics before adding any controller.

### Fixed-Gain PID Baseline (Phase 2-3)

We implemented four cascaded PID loops — altitude (z), roll (φ), 
pitch (θ), and yaw (ψ) — using gains from the literature:

| Loop | Kp | Ki | Kd |
|------|----|----|-----|
| Altitude | 35 | 28 | 6 |
| Attitude | 7 | 10 | 2 |

A discrete mass step from 0.5 kg to 0.8 kg at t=5s produced 
a peak altitude deviation of ~2.3m with no full recovery within 
10s. A more extreme 0.5→5 kg step caused unrealistic thrust 
spikes, highlighting the limits of fixed-gain control.

[INSERT: Phase 3 scope screenshot here]

### Adaptive Gain Scheduling (Phase 4)

We replaced fixed PID gains with 1-D lookup tables that 
interpolate gains based on current mass. The mass signal 
(from the payload Step block) feeds directly into all lookup 
tables, enabling real-time gain adaptation.

**Gain scheduling table:**

| Mass (kg) | Kp_z | Kd_z | Ki_z | Kp_att | Kd_att |
|-----------|------|------|------|--------|--------|
| 0.5 | 35 | 6 | 3 | 7 | 2 |
| 0.8 | 50 | 9 | 5 | 10 | 3 |
| 1.5 | 70 | 13 | 7 | 14 | 4.5 |
| 3.0 | 100 | 18 | 10 | 19 | 6.5 |
| 5.0 | 140 | 25 | 14 | 25 | 9 |

Gains scale roughly proportionally with mass: heavier drones 
require greater thrust authority (higher Kp) and increased 
damping (higher Kd) to handle the additional inertia.

**Feedforward compensation:** A feedforward term U1_ff = m·g 
was added directly to the altitude PID output, pre-compensating 
for gravity. This reduces the reactive burden on the PID 
integrator and decreases transient response time.

**Anti-windup:** Integrator saturation limits (±50) were applied 
to the altitude PID to prevent integral windup during sustained 
payload phases.

[INSERT: Phase 4 scope screenshot here]

### Realistic Disturbances (Phase 5)

To improve simulation fidelity, two additional disturbance 
models were added:

**Asymmetric package drop:** Brief opposing torque pulses on 
roll (U2) and pitch (U3) axes at the moment of package release 
simulate the physical asymmetry of a package shifting weight 
during drop. Pulses last 2 seconds and are implemented as 
paired Step blocks.

**Air turbulence:** Band-limited white noise (power=0.01, 
sample time=0.01s) with independent random seeds was injected 
into U1, U2, and U3, simulating wind gusts and sensor noise.

[INSERT: Phase 5 scope screenshot here]

---

## Results

### Comparison: Fixed PID vs Adaptive PID

| Metric | Fixed PID | Adaptive PID |
|--------|-----------|--------------|
| Peak altitude after pickup (0.5→0.8 kg) | ~2.3m | ~1.3m |
| Settling time after pickup | >10s (not settled) | ~15s |
| Steady-state error | Yes (~0.3m drift) | None |
| Recovery after drop-off | Diverging | Converging |
| Handles 5kg payload | Breaks down | Stable |

The adaptive controller reduced peak overshoot by ~43% and 
eliminated steady-state error entirely. The feedforward term 
visibly reduced the initial transient at t=0, and anti-windup 
prevented the large undershoots seen in earlier iterations.

### Disturbance Rejection

With air turbulence enabled, the adaptive controller maintained 
altitude within ±0.05m during steady hover, demonstrating 
adequate disturbance rejection for the noise levels modeled.

The asymmetric drop disturbance caused a brief additional 
deviation at t=20s but the controller recovered within 
approximately 10 seconds.

---

## Reflections

**What worked well:**
- Lookup table gain scheduling was straightforward to implement 
  and showed clear measurable improvement over fixed gains
- The feedforward term (m·g) meaningfully reduced initial 
  transient response
- Anti-windup on the altitude integrator was essential — 
  without it, large undershoots occurred at drop-off
- The two-step mass model (pickup then drop-off) accurately 
  represented a realistic delivery mission profile

**What was challenging:**
- Integral windup during the payload phase caused significant 
  undershoots at drop-off before anti-windup was added
- The 10x mass case (0.5→5 kg) exposed physical limits of 
  the model — thrust demand became unrealistic, which would 
  need actuator saturation modeling in future work
- A MATLAB Java memory crash mid-session caused loss of 
  unsaved work, which was partially recovered from the 
  .slx.err file
- The model was initially locked due to version mismatch 
  between R2025a and R2025b, requiring an upgrade

**What we would do differently:**
- Add actuator saturation limits from the start to prevent 
  unrealistic thrust demands
- Implement continuous mass variation rather than discrete 
  step changes for smoother gain transitions
- Compare against LQR as the professor suggested — our 
  gain scheduling approach is a reasonable middle ground 
  between fixed PID and full LQR

---

## References

1. Azizi, A. et al. (2022). Modeling and PID control of a 
   quadcopter. ResearchGate.
   https://www.researchgate.net/publication/361821001

2. Argentim, L. M. et al. (2013). PID, LQR and LQR-PID on 
   a quadcopter platform. ResearchGate.
   https://www.researchgate.net/publication/261212676

3. MathWorks. Nonlinear MPC for quadrotor control.
   https://www.mathworks.com/help/mpc/ug/control-of-quadrotor-using-nonlinear-model-predictive-control.html

4. Sahrir, M. S., & Basri, M. A. M. (2018). Simulation 
   modelling of PD-PID controller for quadrotor attitude 
   and altitude control.the Bitcraze Crazyflie 2.1+.

---

## Methodology

### Quadrotor Dynamics Model

We implemented the Newton-Euler equations of motion in a MATLAB 
Function block within Simulink, tracking 8 states:

- Angular positions: roll (φ), pitch (θ), yaw (ψ)
- Angular rates: φ̇, θ̇, ψ̇  
- Altitude: z
- Vertical velocity: ż

The equations of motion are:

φ̈ = (1/Jx)[(Jy−Jz)θ̇ψ̇ + l·U2]  
θ̈ = (1/Jy)[(Jz−Jx)φ̇ψ̇ + l·U3]  
ψ̈ = (1/Jz)[(Jx−Jy)φ̇θ̇ + U4]  
z̈ = (1/m)[cos(φ)cos(θ)·U1] − g  

Physical parameters used:
| Parameter | Value |
|-----------|-------|
| Mass (m) | 0.5 kg |
| Arm length (l) | 0.2 m |
| Jx = Jy | 4.85×10⁻³ kg·m² |
| Jz | 8.81×10⁻³ kg·m² |
| Jr | 3.36×10⁻⁵ kg·m² |

**Open-loop validation:** Applying U1 = m·g held altitude 
steady. Increasing U1 caused climb, decreasing caused descent — 
confirming correct dynamics before adding any controller.

### Fixed-Gain PID Baseline (Phase 2-3)

We implemented four cascaded PID loops — altitude (z), roll (φ), 
pitch (θ), and yaw (ψ) — using gains from the literature:

| Loop | Kp | Ki | Kd |
|------|----|----|-----|
| Altitude | 35 | 28 | 6 |
| Attitude | 7 | 10 | 2 |

A discrete mass step from 0.5 kg to 0.8 kg at t=5s produced 
a peak altitude deviation of ~2.3m with no full recovery within 
10s. A more extreme 0.5→5 kg step caused unrealistic thrust 
spikes, highlighting the limits of fixed-gain control.

[INSERT: Phase 3 scope screenshot here]

### Adaptive Gain Scheduling (Phase 4)

We replaced fixed PID gains with 1-D lookup tables that 
interpolate gains based on current mass. The mass signal 
(from the payload Step block) feeds directly into all lookup 
tables, enabling real-time gain adaptation.

**Gain scheduling table:**

| Mass (kg) | Kp_z | Kd_z | Ki_z | Kp_att | Kd_att |
|-----------|------|------|------|--------|--------|
| 0.5 | 35 | 6 | 3 | 7 | 2 |
| 0.8 | 50 | 9 | 5 | 10 | 3 |
| 1.5 | 70 | 13 | 7 | 14 | 4.5 |
| 3.0 | 100 | 18 | 10 | 19 | 6.5 |
| 5.0 | 140 | 25 | 14 | 25 | 9 |

Gains scale roughly proportionally with mass: heavier drones 
require greater thrust authority (higher Kp) and increased 
damping (higher Kd) to handle the additional inertia.

**Feedforward compensation:** A feedforward term U1_ff = m·g 
was added directly to the altitude PID output, pre-compensating 
for gravity. This reduces the reactive burden on the PID 
integrator and decreases transient response time.

**Anti-windup:** Integrator saturation limits (±50) were applied 
to the altitude PID to prevent integral windup during sustained 
payload phases.

[INSERT: Phase 4 scope screenshot here]

### Realistic Disturbances (Phase 5)

To improve simulation fidelity, two additional disturbance 
models were added:

**Asymmetric package drop:** Brief opposing torque pulses on 
roll (U2) and pitch (U3) axes at the moment of package release 
simulate the physical asymmetry of a package shifting weight 
during drop. Pulses last 2 seconds and are implemented as 
paired Step blocks.

**Air turbulence:** Band-limited white noise (power=0.01, 
sample time=0.01s) with independent random seeds was injected 
into U1, U2, and U3, simulating wind gusts and sensor noise.

[INSERT: Phase 5 scope screenshot here]

---

## Results

### Comparison: Fixed PID vs Adaptive PID

| Metric | Fixed PID | Adaptive PID |
|--------|-----------|--------------|
| Peak altitude after pickup (0.5→0.8 kg) | ~2.3m | ~1.3m |
| Settling time after pickup | >10s (not settled) | ~15s |
| Steady-state error | Yes (~0.3m drift) | None |
| Recovery after drop-off | Diverging | Converging |
| Handles 5kg payload | Breaks down | Stable |

The adaptive controller reduced peak overshoot by ~43% and 
eliminated steady-state error entirely. The feedforward term 
visibly reduced the initial transient at t=0, and anti-windup 
prevented the large undershoots seen in earlier iterations.

### Disturbance Rejection

With air turbulence enabled, the adaptive controller maintained 
altitude within ±0.05m during steady hover, demonstrating 
adequate disturbance rejection for the noise levels modeled.

The asymmetric drop disturbance caused a brief additional 
deviation at t=20s but the controller recovered within 
approximately 10 seconds.

---

## Reflections

**What worked well:**
- Lookup table gain scheduling was straightforward to implement 
  and showed clear measurable improvement over fixed gains
- The feedforward term (m·g) meaningfully reduced initial 
  transient response
- Anti-windup on the altitude integrator was essential — 
  without it, large undershoots occurred at drop-off
- The two-step mass model (pickup then drop-off) accurately 
  represented a realistic delivery mission profile

**What was challenging:**
- Integral windup during the payload phase caused significant 
  undershoots at drop-off before anti-windup was added
- The 10x mass case (0.5→5 kg) exposed physical limits of 
  the model — thrust demand became unrealistic, which would 
  need actuator saturation modeling in future work
- A MATLAB Java memory crash mid-session caused loss of 
  unsaved work, which was partially recovered from the 
  .slx.err file
- The model was initially locked due to version mismatch 
  between R2025a and R2025b, requiring an upgrade

**What we would do differently:**
- Add actuator saturation limits from the start to prevent 
  unrealistic thrust demands
- Implement continuous mass variation rather than discrete 
  step changes for smoother gain transitions
- Compare against LQR as the professor suggested — our 
  gain scheduling approach is a reasonable middle ground 
  between fixed PID and full LQR

---

## References

1. Azizi, A. et al. (2022). Modeling and PID control of a 
   quadcopter. ResearchGate.
   https://www.researchgate.net/publication/361821001

2. Argentim, L. M. et al. (2013). PID, LQR and LQR-PID on 
   a quadcopter platform. ResearchGate.
   https://www.researchgate.net/publication/261212676

3. MathWorks. Nonlinear MPC for quadrotor control.
   https://www.mathworks.com/help/mpc/ug/control-of-quadrotor-using-nonlinear-model-predictive-control.html

4. Sahrir, M. S., & Basri, M. A. M. (2018). Simulation 
   modelling of PD-PID controller for quadrotor attitude 
   and altitude control.
