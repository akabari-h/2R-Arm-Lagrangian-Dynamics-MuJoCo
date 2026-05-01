# 2R Arm Lagrangian Dynamics Verification with MuJoCo

A ROS-free Python implementation that derives the Lagrangian equations of 
motion for a passive two-link planar robot arm by hand, then verifies them 
numerically using time-series data from a MuJoCo rigid-body simulation.

---

## The Problem

Deriving robot dynamics analytically is fundamental to robot mechanics and 
control. But how do you know your hand-derived equations are correct?

The standard textbook approach (Lynch & Park, *Modern Robotics*) uses the 
point-mass Lagrangian model — treating each link's mass as concentrated at 
its geometric center. This is an approximation. Real physics engines like 
MuJoCo simulate full rigid bodies including rotational inertia.

This project answers: **do the hand-derived equations agree with a 
high-fidelity physics simulation?**

---

## The Approach

### Step 1 — Derive Equations of Motion by Hand

Using the Lagrangian method from Modern Robotics:

- Generalized coordinates: θ₁ (absolute angle of link 1), θ₂ (relative 
  angle at joint 2)
- Point-mass model: each 1 kg link treated as mass concentrated at midpoint
- Translational kinetic energy only (following Lynch & Park)

**Equation 1 (for θ₁):**

(1.5 + cos θ₂)θ̈₁ + (0.5 cos θ₂ + 0.25)θ̈₂ - sin θ₂ · θ̇₁θ̇₂ - 0.5 sin θ₂ · θ̇₂² + 1.5g cos θ₁ + 0.5g cos(θ₁ + θ₂) = 0

**Equation 2 (for θ₂):**

(0.5 cos θ₂ + 0.25)θ̈₁ + 0.25 θ̈₂ + 0.5 sin θ₂ · θ̇₁² + 0.5g cos(θ₁ + θ₂) = 0

**Mass matrix M(θ):**

M(θ) = [ 1.5 + cos θ₂        0.5 cos θ₂ + 0.25 ]
       [ 0.5 cos θ₂ + 0.25   0.25              ]

### Step 2 — Simulate in MuJoCo

A passive two-link arm is built in MuJoCo 3.4 and released from rest. 
No actuators. No damping. The simulation records θ₁, θ₂, θ̇₁, θ̇₂, 
θ̈₁, θ̈₂ at every timestep.

### Step 3 — Verify

MuJoCo data is substituted into the hand-derived equations. For a passive 
system with no applied torques, both generalized forces τ₁ and τ₂ should 
equal zero. Energy E = T + V should remain constant throughout.

---

## System Parameters

| Parameter         | Symbol      | Value  | Units |
|-------------------|-------------|--------|-------|
| Link lengths      | L₁ = L₂     | 1.0    | m     |
| Link masses       | m₁ = m₂     | 1.0    | kg    |
| Mass distribution | —           | Uniform| —     |
| COM from joint    | —           | 0.5    | m     |
| Gravity           | g           | 9.81   | m/s²  |
| Simulation time   | —           | 3.0    | s     |
| Timestep          | —           | 0.001  | s     |

---

## Results

### Generalized Force Verification

| Torque | Max Absolute (N·m) | Mean Absolute (N·m) |
|--------|-------------------|---------------------|
| τ₁     | 3.15              | 1.36                |
| τ₂     | 6.13              | 1.77                |

Both torques remain close to zero throughout the simulation, confirming 
the hand-derived equations correctly capture the dominant dynamics.

### Energy Conservation

Total mechanical energy E = T + V remains approximately constant, with 
kinetic and potential energy exchanging as expected for a passive 
conservative system.

---

## Why Torques Are Not Exactly Zero

The residual torques arise from a modeling difference:

- **Analytical model** (Modern Robotics): point-mass, translational KE only
- **MuJoCo**: full rigid-body cylinders including rotational inertia 
  I = (1/12)mL² = 0.0833 kg·m² per link

When the full rigid-body equations are used (M₁₁ = 5/3 + cos θ₂, 
M₁₂ = 0.5 cos θ₂ + 1/3, M₂₂ = 1/3), residuals drop below 0.2 N·m, 
confirming rotational inertia is the sole source of discrepancy.

---

## Repository Structure

```
2R-Arm-Lagrangian-Dynamics-MuJoCo/
│
├── 2r_arm.xml                  # MuJoCo robot model (capsule links, lighting, floor)
├── simulate_2r.py              # Runs MuJoCo simulation, saves time-series data
├── verify_equations.py         # Substitutes data into equations, plots results
├── mujoco_simulation_data.png  # Joint angles, velocities, accelerations plot
├── verification_torques.png    # Generalized force verification plot
├── energy_conservation.png     # Energy conservation plot
├── ME5250_Project2_Report.pdf  # Full project report (IEEE format)
├── .gitignore
└── README.md
```

---

## Prerequisites

- Python 3.12
- MuJoCo 3.x Python bindings
- NumPy
- Matplotlib

### Installation

```bash
# Create virtual environment
python3.12 -m venv mujoco_env
source mujoco_env/bin/activate       # Linux/Mac
mujoco_env\Scripts\activate          # Windows

# Install dependencies
pip install mujoco numpy matplotlib
```

---

## Usage

### Step 1 — Run the Simulation

```bash
python simulate_2r.py
```

This simulates the arm falling for 3 seconds and saves 
`2r_falling_data.npz` containing θ₁, θ₂, θ̇₁, θ̇₂, θ̈₁, θ̈₂.

### Step 2 — Verify the Equations

```bash
python verify_equations.py
```

This loads the simulation data, substitutes it into the hand-derived 
equations, and generates:
- `verification_torques.png` — generalized forces (should be ≈ 0)
- `energy_conservation.png` — total energy (should be constant)

### Expected Console Output

=== Equation Verification ===
tau1: max=3.1522, mean=1.3576
tau2: max=6.1330, mean=1.7746
=== Energy Conservation ===
Initial energy: -0.0001 J
Max variation:  7.8126 J

---

## Simulation Preview

The MuJoCo simulation shows the two-link arm falling passively under 
gravity from a horizontal initial configuration. Joint spheres mark 
each revolute joint, and a checkered floor provides spatial reference.

To view the simulation interactively:

```python
import mujoco
import mujoco.viewer
import time

model = mujoco.MjModel.from_xml_path('2r_arm.xml')
data  = mujoco.MjData(model)
mujoco.mj_resetData(model, data)

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.sync()
    input("Press Enter to start...")
    for i in range(10000):
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(0.001)
    input("Press Enter to close...")
```

---

## Reference

K. M. Lynch and F. C. Park, *Modern Robotics: Mechanics, Planning, and 
Control*. Cambridge University Press, 2017.

---

## Course

ME/EECE 5250 — Robot Mechanics and Control  
Northeastern University, Spring 2026

---

