import numpy as np
import matplotlib.pyplot as plt

data = np.load('2r_falling_data.npz')
t    = data['time']
t1   = data['theta1']
t2   = data['theta2']
t1d  = data['theta1_dot']
t2d  = data['theta2_dot']
t1dd = data['theta1_ddot']
t2dd = data['theta2_ddot']

g = 9.81

# ============================================================
# PART 1: Verify Equations of Motion
# FIX: Gravity terms are NEGATIVE (MuJoCo positive θ = downward)
# ============================================================
tau1 = ((1.5 + np.cos(t2)) * t1dd
      + (0.5*np.cos(t2) + 0.25) * t2dd
      - np.sin(t2) * t1d * t2d
      - 0.5*np.sin(t2) * t2d**2
      - 1.5*g*np.cos(t1)
      - 0.5*g*np.cos(t1 + t2))

# Equation 2: tau2 = M21*ddq1 + M22*ddq2 + C2 + G2
tau2 = ((0.5*np.cos(t2) + 0.25) * t1dd
      + 0.25 * t2dd
      + 0.5*np.sin(t2) * t1d**2
      - 0.5*g*np.cos(t1 + t2))
# tau1 = ((5/3 + np.cos(t2)) * t1dd
#       + (0.5*np.cos(t2) + 1/3) * t2dd
#       - np.sin(t2) * t1d * t2d
#       - 0.5*np.sin(t2) * t2d**2
#       - 1.5*g*np.cos(t1)          # NEGATIVE sign
#       - 0.5*g*np.cos(t1 + t2))    # NEGATIVE sign

# tau2 = ((0.5*np.cos(t2) + 1/3) * t1dd
#       + (1/3) * t2dd
#       + 0.5*np.sin(t2) * t1d**2
#       - 0.5*g*np.cos(t1 + t2))    # NEGATIVE sign

print("=== Equation Verification ===")
print(f"tau1: max={np.max(np.abs(tau1)):.4f}, mean={np.mean(np.abs(tau1)):.4f}")
print(f"tau2: max={np.max(np.abs(tau2)):.4f}, mean={np.mean(np.abs(tau2)):.4f}")
print("(Both should be close to 0 for passive system)")

# ============================================================
# PART 2: Energy Conservation
# FIX: PE is NEGATIVE (arm falls = positive θ = losing height)
# ============================================================

T = ((0.75 + 0.5*np.cos(t2)) * t1d**2
   + (0.5*np.cos(t2) + 0.25) * t1d * t2d
   + 0.125 * t2d**2)

V = -1.5*g*np.sin(t1) - 0.5*g*np.sin(t1 + t2)   # NEGATIVE signs

E = T + V

print("\n=== Energy Conservation ===")
print(f"Initial energy: {E[0]:.4f} J")
print(f"Final energy:   {E[-1]:.4f} J")
print(f"Max variation:  {np.max(E) - np.min(E):.4f} J")
print(f"Relative drift: {(np.max(E)-np.min(E))/(np.max(np.abs(E))+1e-10)*100:.4f}%")

# ============================================================
# PLOT 1: Generalized Forces
# ============================================================
fig1, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))

ax1.plot(t, tau1, 'b-', linewidth=1.5, label='Computed τ₁')
ax1.axhline(0, color='r', linestyle='--', linewidth=1.5, label='Zero reference')
ax1.set_ylabel('τ₁ (N·m)', fontsize=12)
ax1.set_title('Verification: Generalized Forces (should be ≈ 0)', fontsize=13)
ax1.grid(True)
ax1.legend()

ax2.plot(t, tau2, 'g-', linewidth=1.5, label='Computed τ₂')
ax2.axhline(0, color='r', linestyle='--', linewidth=1.5, label='Zero reference')
ax2.set_ylabel('τ₂ (N·m)', fontsize=12)
ax2.set_xlabel('Time (s)', fontsize=12)
ax2.grid(True)
ax2.legend()

plt.tight_layout()
plt.savefig('verification_torques.png', dpi=150)
print("\nSaved: verification_torques.png")

# ============================================================
# PLOT 2: Energy Conservation
# ============================================================
fig2, ax = plt.subplots(figsize=(10, 4))

ax.plot(t, T, 'b-',  linewidth=1.5, label='Kinetic Energy T')
ax.plot(t, V, 'r-',  linewidth=1.5, label='Potential Energy V')
ax.plot(t, E, 'k--', linewidth=2,   label='Total Energy E = T + V')
ax.set_xlabel('Time (s)', fontsize=12)
ax.set_ylabel('Energy (J)', fontsize=12)
ax.set_title('Energy Conservation: Total Energy Should Be Constant', fontsize=13)
ax.legend()
ax.grid(True)

plt.tight_layout()
plt.savefig('energy_conservation.png', dpi=150)
print("Saved: energy_conservation.png")

plt.show()