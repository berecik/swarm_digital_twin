# Physics Engine — Detailed Equations & Models

**Reference:** Valencia et al., "An Open-source UAV Digital Twin framework," *J. Intell. & Robot. Syst.* 111:71 (2025). DOI: 10.1007/s10846-025-02276-7

This document provides the full mathematical derivation and parameter descriptions for every physics model implemented in `simulation/drone_physics.py` and supporting modules.

---

## 1. State Representation

The drone state at time $t$ is:

$$ \mathbf{x}(t) = \bigl[\, \mathbf{p},\; \mathbf{v},\; R,\; \mathbf{\omega} \,\bigr]$$

| Symbol | Dimension | Frame | Description |
|:---|:---:|:---:|:---|
| $\mathbf{p} = [x, y, z]^T$ | 3 | World (ENU) | Position [m] |
| $\mathbf{v} = [v_x, v_y, v_z]^T$ | 3 | World (ENU) | Velocity [m/s] |
| $R$ | 3x3 | Body -> World | Rotation matrix (SO(3)) |
| $\mathbf{\omega} = [p, q, r]^T$ | 3 | Body | Angular velocity [rad/s] |

The rotation matrix $R$ maps body-frame vectors to the world frame. It is parametrised by Euler angles $(\phi, \theta, \psi)$ (roll, pitch, yaw) using the ZYX convention:

$$R = R_z(\psi)\, R_y(\theta)\, R_x(\phi)$$

$$R = \begin{bmatrix}
c_\psi c_\theta & c_\psi s_\theta s_\phi - s_\psi c_\phi & c_\psi s_\theta c_\phi + s_\psi s_\phi \\
s_\psi c_\theta & s_\psi s_\theta s_\phi + c_\psi c_\phi & s_\psi s_\theta c_\phi - c_\psi s_\phi \\
-s_\theta & c_\theta s_\phi & c_\theta c_\phi
\end{bmatrix}$$

where $c_x = \cos(x)$, $s_x = \sin(x)$.

**Re-orthogonalisation:** After each integration step the rotation matrix accumulates numerical error. We restore $R \in SO(3)$ using SVD: $R \leftarrow U V^T$ where $U \Sigma V^T = \text{SVD}(R)$.

---

## 2. Translational Dynamics

### 2.1 World-Frame Formulation (Legacy — Linear Drag)

When `AeroCoefficients` is not set, forces are computed in the world frame:

$$m \dot{\mathbf{v}} = \mathbf{F}_g + \mathbf{F}_T + \mathbf{F}_d + \mathbf{F}_w$$

| Force | Formula | Description |
|:---|:---|:---|
| Gravity | $\mathbf{F}_g = [0, 0, -mg]^T$ | Weight ($g = 9.81$ m/s^2) |
| Thrust | $\mathbf{F}_T = R \cdot [0, 0, T]^T$ | Thrust along body z-axis |
| Linear drag | $\mathbf{F}_d = -k_d \cdot \mathbf{v}$ | Opposes velocity (world frame) |
| Wind | $\mathbf{F}_w$ | From `WindField.get_force()` |

Integration (semi-implicit Euler):

$$\mathbf{v}_{t+1} = \mathbf{v}_t + \frac{\mathbf{F}}{m} \Delta t$$
$$\mathbf{p}_{t+1} = \mathbf{p}_t + \mathbf{v}_t \Delta t + \tfrac{1}{2} \frac{\mathbf{F}}{m} \Delta t^2$$

### 2.2 Body-Frame Formulation (Paper Eq. 3 — Quadratic Drag)

When `AeroCoefficients` is set, Newton's second law is formulated in the body frame following the paper's Eq. 3:

$$\begin{bmatrix} \dot{u} \\ \dot{v} \\ \dot{w} \end{bmatrix} =
\begin{bmatrix} rv - qw \\ pw - ru \\ qu - pv \end{bmatrix} +
\frac{1}{m} \begin{bmatrix} f_x \\ f_y \\ f_z \end{bmatrix}$$

where $(u, v, w) = R^T \mathbf{v}$ are body-frame velocity components and $(f_x, f_y, f_z)$ are total body-frame forces. The first term is the Coriolis/centripetal coupling: $-\mathbf{\omega} \times \mathbf{V}_b$.

Body-frame forces:

$$\mathbf{F}_b = \underbrace{R^T \mathbf{F}_g}_{\text{gravity}} + \underbrace{[0, 0, T]^T}_{\text{thrust}} + \underbrace{\mathbf{F}_{D,b}}_{\text{quadratic drag}} + \underbrace{R^T \mathbf{F}_w}_{\text{wind}}$$

After integration in the body frame, velocity is transformed back to world: $\mathbf{v} = R \, \mathbf{V}_b$.

---

## 3. Quadratic Aerodynamic Drag (Paper Eq. 5)

The aerodynamic drag force in the body frame is:

$$\mathbf{F}_D = -\frac{1}{2} \rho A C_D |\mathbf{V}_b|^2 \hat{\mathbf{V}}_b$$

where:

| Parameter | Symbol | Default | Unit | Description |
|:---|:---:|:---:|:---:|:---|
| Air density | $\rho$ | 1.225 | kg/m^3 | From `Atmosphere` model |
| Reference area | $A$ | 0.04 | m^2 | Quadrotor frontal area |
| Drag coefficient | $C_D$ | 1.0 | - | Bluff body (quadrotor) |
| Body velocity | $\mathbf{V}_b$ | - | m/s | $R^T \mathbf{v}$ |
| Velocity unit vector | $\hat{\mathbf{V}}_b$ | - | - | $\mathbf{V}_b / |\mathbf{V}_b|$ |

**Key property:** Drag scales with velocity squared. Doubling speed quadruples drag force.

**Terminal velocity:** At terminal velocity, drag equals weight:

$$\frac{1}{2} \rho A C_D V_t^2 = mg \implies V_t = \sqrt{\frac{2mg}{\rho A C_D}}$$

For the default quadrotor ($m=1.5$ kg, $A=0.04$ m^2, $C_D=1.0$, $\rho=1.225$):

$$V_t = \sqrt{\frac{2 \times 1.5 \times 9.81}{1.225 \times 0.04 \times 1.0}} \approx 24.5 \text{ m/s}$$

---

## 4. ISA Atmosphere Model

Air density varies with altitude above mean sea level (MSL) following the International Standard Atmosphere (ISA) troposphere formula:

$$\rho(h) = \rho_0 \left(1 - 2.2558 \times 10^{-5} \cdot h\right)^{4.2559}$$

| Altitude (MSL) | $\rho$ (kg/m^3) | Ratio to sea level |
|:---:|:---:|:---:|
| 0 m | 1.225 | 100% |
| 1000 m | 1.112 | 90.8% |
| 2500 m | 0.957 | 78.1% |
| 4500 m | 0.770 | 62.8% |

**Effect on drag:** At 4500 m (paper's Andean test site), drag force is ~63% of sea level. This means higher cruise speeds and longer braking distances.

---

## 5. Wind Perturbation Model (Paper Eq. 5-7)

Wind produces an additional aerodynamic force on the drone:

$$\Delta F_D = \frac{1}{2} \rho A C_D |\mathbf{V}_{wind}|^2 \cdot \hat{\mathbf{V}}_{wind}$$

$$\Delta F_L = \frac{1}{2} \rho A C_L |\mathbf{V}_{wind}|^2 \cdot \hat{\mathbf{n}}$$

$$\Delta F_W = ||\mathbf{F}_D + \mathbf{F}_L||$$

For quadrotors, $C_L \approx 0$ so the lift perturbation is negligible and wind force is purely drag.

### Wind Types

**Constant wind:** A uniform velocity field $\mathbf{V}_w = V_w \cdot \hat{\mathbf{d}}$ where $\hat{\mathbf{d}}$ is the wind direction unit vector.

**Dryden turbulence (MIL-F-8785C):** A first-order Markov process driven by white noise:

$$\dot{\mathbf{x}} = -\frac{V}{L} \mathbf{x} + \sqrt{\frac{2V}{L}} \mathbf{\eta}(t)$$

where $L$ is the turbulence scale length (altitude-dependent), $V$ is the mean wind speed, and $\mathbf{\eta}$ is Gaussian white noise scaled by gust intensity.

Scale lengths (low altitude):
- Longitudinal/lateral: $L_u = L_v = h / (0.177 + 0.000823 \cdot h)^{1.2}$
- Vertical: $L_w = h$

**From flight log:** Wind velocity interpolated from an altitude profile: $V_w(t) = \text{interp}(t, t_{log}, V_{log})$.

---

## 6. Rotational Dynamics (Paper Eq. 4)

Angular acceleration follows Euler's rotation equation:

$$J \dot{\mathbf{\omega}} = \mathbf{\tau} - \mathbf{\omega} \times (J \mathbf{\omega}) + \mathbf{\tau}_{drag}$$

$$\dot{\mathbf{\omega}} = J^{-1} \bigl[\mathbf{\tau} - \mathbf{\omega} \times (J \mathbf{\omega}) + \mathbf{\tau}_{drag}\bigr]$$

where:

| Symbol | Description |
|:---|:---|
| $J$ | Full 3x3 inertia tensor (may have off-diagonal terms) |
| $\mathbf{\tau}$ | Applied torque from controller [N*m] |
| $\mathbf{\omega} \times (J\mathbf{\omega})$ | Gyroscopic coupling term |
| $\mathbf{\tau}_{drag} = -k_\omega \mathbf{\omega}$ | Angular drag torque |

### Inertia Tensor

The general inertia tensor is:

$$J = \begin{bmatrix}
J_{xx} & -J_{xy} & -J_{xz} \\
-J_{xy} & J_{yy} & -J_{yz} \\
-J_{xz} & -J_{yz} & J_{zz}
\end{bmatrix}$$

For symmetric quadrotors, off-diagonal terms are zero and $J_{xx} \approx J_{yy}$. Non-symmetric configurations or sensor payloads introduce cross-coupling through $J_{xy}, J_{xz}, J_{yz}$.

**Gyroscopic coupling example:** A quadrotor spinning in yaw ($r \neq 0$) with off-diagonal $J_{xy} \neq 0$ will experience roll/pitch disturbances proportional to the product of inertia.

### Rotation Integration

The rotation matrix is updated using a first-order approximation:

$$R_{t+1} = R_t \cdot (I + [\mathbf{\omega}]_\times \Delta t)$$

where $[\mathbf{\omega}]_\times$ is the skew-symmetric matrix:

$$[\mathbf{\omega}]_\times = \begin{bmatrix}
0 & -r & q \\
r & 0 & -p \\
-q & p & 0
\end{bmatrix}$$

---

## 7. Terrain Model

The terrain is stored as a 2D elevation grid $z = f(x, y)$ over a rectangular region.

### Elevation Query (Bilinear Interpolation)

For a query point $(x, y)$, the grid indices are:

$$i_x = \lfloor (x - x_0) / \Delta \rfloor, \quad i_y = \lfloor (y - y_0) / \Delta \rfloor$$

where $(x_0, y_0)$ is the grid origin and $\Delta$ is the resolution. The interpolated elevation is:

$$z(x,y) = (1-s_x)(1-s_y) z_{00} + s_x(1-s_y) z_{10} + (1-s_x) s_y z_{01} + s_x s_y z_{11}$$

where $s_x, s_y \in [0,1)$ are the fractional grid coordinates.

### Ground Collision

At each physics step, the ground constraint is:

$$\text{if } z_{drone} < z_{terrain}(x, y): \quad z_{drone} \leftarrow z_{terrain}, \quad v_z \leftarrow \max(v_z, 0)$$

### Terrain Sources

| Constructor | Source | Use case |
|:---|:---|:---|
| `TerrainMap.flat(elev)` | Constant elevation | Baseline testing |
| `TerrainMap.from_array(grid)` | NumPy 2D array | Programmatic terrain |
| `TerrainMap.from_function(fn)` | Analytical $z = f(x,y)$ | Scenario generation |
| `TerrainMap.from_stl(path)` | Binary STL mesh | BlenderGIS/SRTM pipeline |

---

## 8. Cascaded Position Controller

The controller has two loops:

### Outer Loop: Position -> Desired Acceleration

$$a_x^{des} = \text{PID}_x(x^{target} - x)$$
$$a_y^{des} = \text{PID}_y(y^{target} - y)$$
$$a_z^{des} = \text{PID}_z(z^{target} - z) + g$$

The $+g$ term provides gravity compensation so the controller output is the net acceleration above hover.

### Thrust Computation

$$T = m \cdot ||\mathbf{a}^{des}||$$

The desired body z-axis direction is $\hat{z}_{des} = \mathbf{a}^{des} / ||\mathbf{a}^{des}||$.

### Inner Loop: Attitude Error -> Torque

The desired rotation matrix $R_{des}$ is constructed from $\hat{z}_{des}$ and the target yaw angle. The attitude error is extracted from the skew-symmetric part of $R^T R_{des}$:

$$e_{roll} = (R_{err})_{21} - (R_{err})_{12}$$
$$e_{pitch} = (R_{err})_{02} - (R_{err})_{20}$$
$$e_{yaw} = (R_{err})_{10} - (R_{err})_{01}$$

These errors drive three independent PID controllers that output torque commands.

### PID Gains

| Loop | $K_p$ | $K_i$ | $K_d$ | Limit |
|:---|:---:|:---:|:---:|:---:|
| Position X/Y | 4.0 | 0.5 | 3.0 | 8.0 m/s^2 |
| Position Z | 6.0 | 1.0 | 4.0 | 15.0 m/s^2 |
| Roll/Pitch | 8.0 | 0.1 | 2.0 | max_torque |
| Yaw | 4.0 | 0.05 | 1.0 | max_torque |

---

## 9. Aerodynamic Lift (Paper Eq. 6) and Fixed-Wing Stall Model

### 9.1 Lift Force

Aerodynamic lift acts perpendicular to the velocity vector in the body XZ-plane:

$$\mathbf{F}_L = \frac{1}{2} \rho A \, C_L(\alpha) \, |\mathbf{V}_b|^2 \, \hat{\mathbf{L}}$$

where $\hat{\mathbf{L}}$ is the lift direction: the body-frame velocity vector rotated +90° in the XZ plane.

$$\hat{\mathbf{L}} = \frac{[-\hat{V}_z, \; 0, \; \hat{V}_x]}{|[-\hat{V}_z, \; 0, \; \hat{V}_x]|}$$

For quadrotors, $C_L = 0$ so no lift is generated. For fixed-wing aircraft, $C_L$ is a function of angle of attack $\alpha$.

### 9.2 Angle of Attack

The angle of attack is computed from the body-frame velocity:

$$\alpha = \arctan\!\left(\frac{-V_z^b}{V_x^b}\right)$$

where $V_x^b$ is the forward body velocity and $V_z^b$ is the upward body velocity.

### 9.3 Fixed-Wing Aerodynamic Model (`FixedWingAero`)

Following the paper's Table 3 and Fig. 5, the fixed-wing model uses piecewise-linear lift and quadratic drag curves with a stall transition:

**Pre-stall** ($|\alpha| < \alpha_{stall}$):

$$C_L(\alpha) = C_{L\alpha} \cdot (\alpha - \alpha_0)$$
$$C_D(\alpha) = C_{D0} + C_{D\alpha} \cdot \alpha^2$$

**Post-stall** ($|\alpha| \geq \alpha_{stall}$):

$$C_L(\alpha) = C_{L\alpha,stall} \cdot (\alpha - \alpha_0)$$
$$C_D(\alpha) = C_{D0} + C_{D\alpha,stall} \cdot \alpha^2$$

| Parameter | Symbol | Default | Unit | Description |
|:---|:---:|:---:|:---:|:---|
| Zero-lift AoA | $\alpha_0$ | 0.05236 | rad (~3°) | AoA at which $C_L = 0$ |
| Stall angle | $\alpha_{stall}$ | 0.26180 | rad (~15°) | Transition to post-stall regime |
| Parasitic drag | $C_{D0}$ | 0.02 | - | Drag at zero AoA |
| Lift slope (pre-stall) | $C_{L\alpha}$ | 3.50141 | 1/rad | Lift curve slope before stall |
| Drag slope (pre-stall) | $C_{D\alpha}$ | 0.63662 | 1/rad² | Induced drag slope before stall |
| Lift slope (post-stall) | $C_{L\alpha,stall}$ | -1.1459 | 1/rad | Negative — lift drops after stall |
| Drag slope (post-stall) | $C_{D\alpha,stall}$ | 2.29183 | 1/rad² | Drag increases sharply after stall |

**Stall behaviour:** At $\alpha > \alpha_{stall}$, the lift slope becomes negative (lift decreases) while drag increases sharply. This models the flow separation that occurs when the wing exceeds its critical angle of attack.

---

## 10. Airframe Presets

| Preset | Mass | Inertia | Aero | Use case |
|:---|:---:|:---|:---|:---|
| `make_generic_quad()` | 1.5 kg | diag(0.02, 0.02, 0.04) | None (linear drag) | Fast prototyping |
| `make_holybro_x500()` | 2.0 kg | diag(0.03, 0.03, 0.05) | $C_D=1.1$, $A=0.06$ m^2 | Paper validation |
| `make_fixed_wing()` | 3.0 kg | with off-diag products | `FixedWingAero` ($A=0.50$ m^2) | Fixed-wing simulation |

### 10.1 Aerodynamic Parameter Registry (Calibration Provenance)

| Preset | Parameter set | Source / provenance | Valid runtime range | Uncertainty notes |
|:---|:---|:---|:---|:---|
| `make_generic_quad()` | `mass=1.5 kg`, linear drag path | Engineering baseline estimate (project default) | `mass` in [0.5, 30.0] kg | Representative only; not tuned to a specific frame |
| `make_holybro_x500()` | `A=0.06 m^2`, `C_D=1.1`, ISA atmosphere | Paper-aligned calibration + project tuning for X500 class | `A` in [0.01, 1.0] m², `C_D` in [0.2, 2.5] | Depends on payload, prop guards, and wind exposure |
| `make_fixed_wing()` | `A=0.50 m^2`, `alpha_0=0.05236`, `alpha_stall=0.26180`, `C_La/C_Da` piecewise | Valencia et al. (2025) Table 3 / Fig. 5 mapped into `FixedWingAero` | `alpha_0` in [-0.15, 0.20] rad, `alpha_stall` in [0.10, 0.60] rad, `C_La_stall < 0` | High sensitivity near stall; post-stall fit is model-level approximation |

Runtime guardrails are enforced in `simulation/drone_physics.py`: when an aerodynamic preset is outside validated ranges, the simulator emits a `RuntimeWarning` once per `DroneParams` object.

---

## 11. MAVLink Bridge

The `mavlink_bridge.py` module provides a UDP MAVLink v2 bridge connecting the standalone simulator to QGroundControl or other MAVLink-compatible ground control stations.

### Messages Sent (Sim → GCS)

| Message | ID | Content | Rate |
|:---|:---:|:---|:---:|
| `HEARTBEAT` | 0 | Vehicle type, armed state, mode | 1 Hz |
| `ATTITUDE` | 30 | Roll, pitch, yaw + rates | Per sim step |
| `GLOBAL_POSITION_INT` | 33 | GPS (lat/lon/alt), velocity (NED) | Per sim step |
| `VFR_HUD` | 74 | Airspeed, groundspeed, heading, throttle, altitude, climb rate | Per sim step |
| `SYS_STATUS` | 1 | Battery voltage, current, remaining % | Per sim step |

### Messages Received (GCS → Sim)

| Message | ID | Content |
|:---|:---:|:---|
| `COMMAND_LONG` | 76 | Arm/disarm, mode change |
| `SET_POSITION_TARGET_LOCAL_NED` | 84 | Guided-mode waypoints |

### Coordinate Conversion (ENU → GPS)

The simulator uses ENU (East-North-Up) internally. For MAVLink GPS messages, positions are converted:

$$\text{lat} = \text{lat}_{ref} + \frac{y_{ENU}}{111320}$$
$$\text{lon} = \text{lon}_{ref} + \frac{x_{ENU}}{111320 \cdot \cos(\text{lat}_{ref})}$$
$$\text{alt}_{MSL} = \text{alt}_{ref} + z_{ENU}$$

Default reference: Zurich (47.3769°N, 8.5417°E, 408 m MSL).

### MAVLink v2 Wire Format

Each message uses the MAVLink v2 framing: `0xFD` start byte, 10-byte header (payload length, flags, sequence, system/component ID, 3-byte message ID), payload, X.25 CRC with message-specific CRC extra byte.

---

## 12. Validation Methodology

Following Valencia et al. (2025) Table 5 and Fig. 13, simulation accuracy is measured by:

$$\text{RMSE}_x = \sqrt{\frac{1}{N} \sum_{i=1}^{N} (x_i^{sim} - x_i^{ref})^2}$$

$$\text{RMSE}_{total} = \sqrt{\frac{1}{N} \sum_{i=1}^{N} ||\mathbf{p}_i^{sim} - \mathbf{p}_i^{ref}||^2}$$

Additional metrics: median error, 25th/75th percentiles, maximum error. These are computed by `validation.compute_rmse()`.

**Paper benchmark results:**
- Quadrotor: RMSE < 0.1 m
- Fixed-wing: RMSE < 2 m (8-9% of flight envelope)

---

## 13. Numerical Integration Notes

- **Timestep:** Default $\Delta t = 0.005$ s (200 Hz). Smaller timesteps improve accuracy at the cost of computation.
- **Integration scheme:** Semi-implicit Euler (velocity update then position update using new velocity contribution).
- **Energy conservation:** With zero drag, the integrator conserves total mechanical energy ($KE + PE$) within 0.5% over 0.5 s of freefall at $\Delta t = 0.001$ s.
- **Rotation drift:** SVD re-orthogonalisation keeps $R \in SO(3)$ after every step, preventing accumulated drift from violating $R R^T = I$.

---

## 14. Units and Conventions

| Quantity | Unit | Convention |
|:---|:---:|:---|
| Position | m | World ENU: X=East, Y=North, Z=Up |
| Velocity | m/s | World frame |
| Angles | rad | Roll ($\phi$), Pitch ($\theta$), Yaw ($\psi$) |
| Angular velocity | rad/s | Body frame (p=roll rate, q=pitch rate, r=yaw rate) |
| Force | N | |
| Torque | N*m | Body frame |
| Mass | kg | |
| Inertia | kg*m^2 | Body frame |
| Air density | kg/m^3 | |
| Area | m^2 | Reference (frontal) area |
| Time | s | |
