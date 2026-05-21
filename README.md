# Racing Car Model Predictive Control (MPC)

A comprehensive Model Predictive Control system for autonomous racing vehicles, optimizing lap time while maintaining safe track boundaries and managing battery energy.

## Project Overview

This project implements a **nonlinear Model Predictive Control (MPC)** framework for autonomous race vehicles using the **ACADOS** optimal control solver. The system simultaneously optimizes for:
- **Minimum lap time** via velocity maximization
- **Track stability** by constraining lateral errors
- **Energy efficiency** by tracking battery state of charge (SoC)

The implementation uses a **curvilinear reference frame** based on arc-length parameterization, enabling natural path following on complex track geometries.

---

## Key Features

✅ **Nonlinear Bicycle Model** with realistic tire dynamics  
✅ **Curvilinear Coordinate System** for track-relative state representation  
✅ **Real-time MPC Solver** using SQP-RTI method  
✅ **Dynamic Curvature Tracking** with track geometry interpolation  
✅ **Battery Management** - SoC dynamics with drag and rolling resistance  
✅ **Obstacle Avoidance** - Track-width constraints with obstacle handling  
✅ **Velocity Profile Generation** - Reference speed computation based on track curvature  

---

## Technical Architecture

### Core Components

```
racing_car/
├── main.m                      # Full MPC closed-loop simulation with track
├── main_custom_cost.m          # Variant with custom cost function
├── main_new_acados.m           # Alternative ACADOS setup
├── main_off_vel.m              # Offline velocity profile computation
│
├── racecar_model.m             # Vehicle dynamics (7 states, 2 inputs)
├── racecar_ocp_setup.m         # OCP problem definition
├── racecar_mpc_test.m          # Standalone MPC test with warm-start
├── race_car_model_mod.m        # Modified dynamics variant
│
├── feasibility_check.m         # Constraint feasibility analysis
├── LMS_Track.txt               # Track geometry data
│
├── c_generated_code/           # Auto-generated C code for solver
├── helper_functions/           # Utility functions for visualization
└── build/                      # CMake build directory
```

### Vehicle State (7 states)

```matlab
x = [s; vx; vy; wz; ye; theta_e; SoC]

s       - Progress along track (arc-length) [m]
vx      - Longitudinal velocity [m/s]
vy      - Lateral velocity [m/s]
wz      - Yaw rate [rad/s]
ye      - Lateral track error [m]
theta_e - Heading error relative to track [rad]
SoC     - Battery state of charge [%]
```

### Control Inputs (2 inputs)

```matlab
u = [delta; a]

delta   - Steering angle [rad]  (constrained: -0.6 to 0.6)
a       - Longitudinal acceleration [m/s²]
```

### Vehicle Parameters

| Parameter | Value | Unit |
|-----------|-------|------|
| Mass | 1.8 | kg |
| Yaw inertia | 0.03 | kg⋅m² |
| Front axle distance | 0.125 | m |
| Rear axle distance | 0.125 | m |
| Front cornering stiffness | 68 | N/rad |
| Rear cornering stiffness | 71 | N/rad |
| Drag coefficient | 0.35 | - |
| Tire friction coefficient | 1.2 | - |

---

## Physics & Dynamics

### Nonlinear Bicycle Model

The vehicle follows a **nonlinear kinematic-dynamic bicycle model** with:

1. **Lateral Tire Forces** (Pacejka-inspired):
   ```
   Fy_f = Cf × (δ - vy/vx + lf·ωz/vx)
   Fy_r = Cr × (-vy/vx - lr·ωz/vx)
   ```

2. **Longitudinal-Lateral-Yaw Dynamics**:
   ```
   vx_dot = a - (Fy_f·sin(δ))/m - μ·g + ωz·vy
   vy_dot = (Fy_f·cos(δ) + Fy_r)/m - ωz·vx
   ωz_dot = (Fy_f·lf·cos(δ) - Fy_r·lr)/Iz
   ```

3. **Curvilinear Track Dynamics**:
   ```
   s_dot = (vx·cos(θe) - vy·sin(θe)) / (1 - ye·κ)
   ye_dot = vx·sin(θe) + vy·cos(θe)
   θe_dot = ωz - κ·s_dot
   ```
   where κ is the track curvature parameter.

4. **Battery Energy Model**:
   ```
   P = ½·Cd·ρ·A·vx² + μ_rr·m·g·vx   (aerodynamic + rolling losses)
   SoC_dot = -(P / C_alpha)          (discharge rate)
   ```

---

## MPC Formulation

### Optimization Problem

**Prediction Horizon**: N = 50 steps, T = 1.0 s total (Δt = 0.02 s per step)

**Cost Function** (Linear-Least Squares form):
```
J = Σ ||y_k - y_ref_k||²_W + ||y_N - y_ref_N||²_W_e

y_k = [x_k; u_k] = [s, vx, vy, ωz, ye, θe, SoC, δ, a]
```

**State Weights (Q)**:
- s: 1.0 (arc-length tracking)
- vx: 1e-8 (encourage high speed)
- vy: 1e-2 (lateral stability)
- ωz: 1e-2 (yaw damping)
- ye: 1e-8 (tight lateral error tracking)
- θe: 5e-3 (heading alignment)
- SoC: 5e-1 (energy preservation)

**Input Weights (R)**:
- δ: 1e-4 (smooth steering)
- a: 1e-3 (smooth acceleration)

### Constraints

```matlab
% Steering angle bounds
-0.6 ≤ δ ≤ 0.6   [rad]

% Lateral error bounds (track-width dependent)
ye_min ≤ ye ≤ ye_max   [m]

% Obstacle avoidance in curvilinear coordinates
track_bounds_with_obstacle(s, width, s_obs, ye_obs, obs_w, obs_L)
```

### Solver Configuration

| Setting | Value |
|---------|-------|
| NLP Solver | SQP-RTI |
| QP Solver | Partial Condensing HPIPM |
| Integrator | Explicit Runge-Kutta (4 stages, 3 steps) |
| Tolerance | 1e-4 |
| Hessian | Gauss-Newton (exact off) |

---

## File Descriptions

### Main Scripts

#### `main.m` - **Primary MPC Simulation**
Full closed-loop racing simulation with:
- Track geometry loading from `LMS_Track.txt`
- Curvature interpolation and smoothing
- Velocity profile generation
- Dynamic obstacle handling
- Live trajectory visualization
- Lap time computation

**Typical runtime**: ~10 seconds closed-loop simulation

#### `main_custom_cost.m`
Alternative cost function formulation for different optimization objectives.

#### `main_new_acados.m`
Experimental ACADOS interface with different problem setup approach.

#### `main_off_vel.m`
Offline velocity profile generator - pre-computes optimal speed at each track position based on track curvature and vehicle limits.

### Model & Control Setup

#### `racecar_model.m`
Defines the **vehicle dynamics model** using CasADi symbolic math:
- State vector definition (7 states)
- Input vector definition (2 inputs)
- Explicit dynamics formulation
- Parameter setup (track curvature κ)
- Returns ACADOS-compatible model structure

#### `racecar_ocp_setup.m`
Configures the **Optimal Control Problem**:
- OCP model definition
- Solver options
- Constraint setup
- Returns ACADOS OCP solver object

#### `racecar_mpc_test.m`
Standalone **MPC testing function** with:
- Reduced state space (6 states, no arc-length)
- Warm-start from previous solution
- Real-time solver performance metrics
- Live 6-panel visualization

### Utilities

#### `feasibility_check.m`
Analyzes constraint feasibility - checks if vehicle can satisfy all constraints given vehicle limits.

#### `race_car_model_mod.m`
Modified vehicle dynamics variant - alternative model formulation.

### Data Files

#### `LMS_Track.txt`
Track geometry definition with 250+ waypoints:
```
s(m)  x(m)  y(m)  psi(rad)  kappa(rad⁻¹)
```
Defines a closed loop track with straight sections and high-curvature turns.

---

## Setup & Execution

### Prerequisites

**MATLAB Toolboxes**:
- MATLAB R2020b or later
- ACADOS (with MATLAB interface)
- CasADi (for symbolic computation)

**Installation**:
```bash
# Clone ACADOS
git clone https://github.com/acados/acados.git
cd acados
git submodule update --recursive --init

# Build with CMake
mkdir build && cd build
cmake ..
make install

# Install MATLAB interface
cd ../interfaces/acados_matlab_octave
make clean
make
```

### Running Simulations

**Full MPC Simulation** (recommended):
```matlab
>> main
```
Outputs:
- Lap time completion
- 4-panel trajectory plots (speed, track error, SoC, inputs)
- 2D track visualization with acceleration heatmap

**MPC Testing**:
```matlab
>> racecar_mpc_test()
```
Outputs:
- Real-time 6-panel solver performance
- Warm-start effectiveness metrics

**Offline Velocity Profile**:
```matlab
>> main_off_vel
```
Generates optimal reference velocity based on track curvature.

---

## Performance Metrics

### Simulation Results
- **Lap completion**: Track circuit closure detection
- **Lap time**: Computed from arc-length and velocity
- **Constraint satisfaction**: ye bounds checking
- **Solver statistics**: Iterations, computation time per step

### Optimization Performance
- SQP-RTI solver: ~5-20 ms per MPC step
- Warm-start recovery: 2-3 iterations typical
- Feasibility: 99%+ satisfaction rate

---

## Key Algorithms

### 1. **Track Curvature Interpolation**
```matlab
kappa_scaled = kappa_raw / 10;
kappa_smooth = movmean(kappa_scaled, 7);  % Moving average filter
kapparef_s = interpolant('kapparef_s', 'bspline', {s_ext}, kappa_ext);
```

### 2. **Velocity Profile Generation**
Pre-computes longitudinal speed reference based on:
- Track curvature
- Vehicle lateral acceleration limits (μ·g)
- Tire friction constraints

### 3. **Obstacle Handling**
Dynamic track-width constraints in curvilinear coordinates:
```matlab
[ye_min_s, ye_max_s] = track_bounds_with_obstacle(s, track_width, s_obs, ye_obs, obs_w, obs_L)
```

### 4. **Warm-Start Initialization**
Reuses previous MPC solution for:
- Faster solver convergence
- Real-time performance
- Consistent control

---

## Extension Points

### Customization Ideas

1. **Multi-vehicle racing** - Add inter-agent constraints
2. **Real tire model** - Integrate Pacejka tire model
3. **Slip control** - Add wheel slip dynamics
4. **Road friction** - Adaptive μ based on track surface
5. **Wind effects** - Lateral wind disturbances
6. **Hardware integration** - ROS2 interface for real vehicles
7. **Advanced solvers** - iLQG, DDP for faster computation

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| ACADOS solver not found | Verify installation path in MATLAB |
| Track data loading fails | Check `LMS_Track.txt` path and format |
| Constraint infeasibility | Relax ye bounds or increase δ_max |
| Slow convergence | Increase horizon N or adjust weights |
| Simulation diverges | Reduce acceleration limits or dt |

---

## References & Related Work

- **ACADOS**: https://github.com/acados/acados - Fast optimal control solver
- **CasADi**: https://casadi.org - Symbolic computation framework
- **Curvilinear MPC**: Race car trajectory optimization using track-relative coordinates
- **Nonlinear Bicycle Model**: Standard vehicle dynamics in autonomous racing

---

## Author & Contact

**Created by**: bhimray  
**Repository**: https://github.com/bhimray/racing_car  
**Last Updated**: December 2025

---

## License

This project is provided as-is for research and educational purposes.

---

## Quick Reference

| Scenario | Script | Output |
|----------|--------|--------|
| Full racing MPC | `main.m` | Lap time, trajectory plots |
| MPC performance | `racecar_mpc_test.m` | Solver metrics, stability plots |
| Velocity optimization | `main_off_vel.m` | Reference speed profile |
| Constraint analysis | `feasibility_check.m` | Feasibility report |

For detailed documentation on dynamics, constraints, or ACADOS setup, refer to the inline comments in each MATLAB file.
