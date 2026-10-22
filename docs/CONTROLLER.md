# Pure Pursuit Controller Design & Stability Analysis — `AME-RoboRacer-Project-YourStudentID`

- **Course / Competition:** AME RoboRacer Competition — GIU Berlin (Winter Semester 2026)
- **Author:** `YourName` (`YourStudentID`)
- **Primary Package:** `src/control/pure_pursuit/` (`controller_core.py`, `pure_pursuit_node.py`)

---

## 1. Exact Ackermann Geometric Tracking Law

Given current vehicle rear/center pose $(x, y, \psi)$ in frame `map`, wheelbase $L = l_f + l_r = 0.15875 + 0.17145 = 0.3302\text{ m}$ (matching `f110_gym` F1TENTH physical chassis parameters), and speed-scaled lookahead target $(x_{\text{la}}, y_{\text{la}})$:

1. **Transform target into vehicle body frame:**
   $$\begin{bmatrix} x_v \\ y_v \end{bmatrix} = \begin{bmatrix} \cos\psi & \sin\psi \\ -\sin\psi & \cos\psi \end{bmatrix} \begin{bmatrix} x_{\text{la}} - x \\ y_{\text{la}} - y \end{bmatrix}, \qquad L_d^2 = \max(x_v^2 + y_v^2, L_{d,\min}^2)$$

2. **Instantaneous Pure Pursuit arc curvature $\gamma$ and exact Ackermann steering angle:**
   $$\gamma = \frac{2 y_v}{L_d^2} = \frac{2 \sin\alpha}{L_d}, \qquad \delta_{\text{geom}} = \arctan(L \cdot \gamma)$$
   Unlike the unmodified starter code (which multiplied $\gamma$ by a heuristic scalar and clamped asymmetrically), $\arctan(L \cdot \gamma)$ yields exact zero steady-state curvature error on circular arcs ($\delta = \arctan(L / R)$ verified to `< 2e-3 rad` in `test_controller_core.py`).

3. **Derivative damping, low-pass filtering, servo rate limiting & symmetric saturation:**
   $$\delta_{\text{raw}} = k_{\text{steer}} \delta_{\text{geom}} + k_p \gamma + k_d \operatorname{clip}(\dot{\gamma}, -8, +8)$$
   $$\delta_{\text{filt}}[k] = \delta[k-1] + \alpha_{\text{lp}}\big(\delta_{\text{raw}}[k] - \delta[k-1]\big)$$
   $$\delta_{\text{cmd}}[k] = \operatorname{clip}\!\left(\operatorname{clip}\!\left(\delta_{\text{filt}}[k],\; \delta[k-1] - \dot{\delta}_{\max}\Delta t,\; \delta[k-1] + \dot{\delta}_{\max}\Delta t\right),\; -\delta_{\max},\; +\delta_{\max}\right)$$
   with exact F1TENTH physical servo bounds $\delta_{\max} = 0.4189\text{ rad}$ (`24.0 deg`) and $\dot{\delta}_{\max} = 3.2\text{ rad/s}$.

---

## 2. Speed-Adaptive Lookahead & Forward Curvature Preview Regulation

1. **Speed-Adaptive Lookahead Distance:**
   - **Linear interpolation mode (`use_linear_lad_interp: true`):** maps current speed $v \in [v_{\min}, v_{\max}]$ smoothly onto $[L_{d,\min}, L_{d,\max}]$, preventing short-lookahead high-speed oscillation on Berlin's fast eastern curve while retaining crisp turn-in through the tight northern hairpin ($R \approx 1.0\text{ m}$).
   - **Gain mode (`use_linear_lad_interp: false`):** $L_d(v) = \operatorname{clip}(L_{d,\min} + k_v v,\; L_{d,\min},\; L_{d,\max})$.

2. **Forward Curvature Preview Longitudinal Speed Control:**
   - Evaluates peak path Menger curvature $\kappa_{\text{preview}} = \max_{s \in [0, S_{\text{prev}}]} |\kappa(s)|$ ahead of the vehicle over preview horizon $S_{\text{prev}} \in [2.4, 3.1]\text{ m}$ so braking begins **before** corner entry:
   $$\kappa_{\text{eff}} = \max(|\gamma|, \kappa_{\text{preview}}), \qquad v_{\text{curve}} = \sqrt{\frac{a_{y,\max}}{\max(\kappa_{\text{eff}}, 10^{-4})}}$$
   $$v_{\text{cmd}} = \operatorname{clip}\!\left(\min\!\left(\frac{v_{\text{path}}}{k_{\text{div}}},\; v_{\text{curve}}\right),\; v_{\min},\; v_{\max}\right)$$
   combined with high-steer skidding attenuation above $\delta_{\text{skid}}$ so cornering lateral tire force never saturates the single-track friction ellipse ($\mu g \approx 10.29\text{ m/s}^2$).

---

## 3. Fail-Safe Architecture & Verified Fault Responses

| Fault / Edge Condition | Detection Mechanism | Controller Action | Verified Result |
| :--- | :--- | :--- | :--- |
| **Emergency Stop (`/pause = True`)** | Subscription on `/pause` (`std_msgs/Bool`) | Immediately publishes `speed = 0.0, steering_angle = 0.0` | `0.000 m/s` cmd within `10 ms`; full stop from `3.24 m/s` in **`0.39 s` (`0.56 m`)** |
| **Sensor / LiDAR Dropout** | `watchdog_node` periodic silence check (`age > 1.0 s`) + `odom_timeout` (`0.5 s`) | `watchdog_node` sets `is_critical = True` & publishes `/pause = True`; `pure_pursuit_node` commands `0.0 m/s` | Both `watchdog_lidar_silence_critical == True` & `speed == 0.0` verified |
| **Empty / Corrupt Reference Path** | `len(path) < 3` or non-finite waypoint | Rejects update, holds safe stop `(0.0, 0.0)` with `reason='invalid_or_empty_path'` | Unit-tested in `test_controller_core.py` |
| **NaN / Non-Finite Vehicle State** | `math.isfinite(...)` check + clamped quaternion `arcsin` | Immediate safe stop `(0.0, 0.0)` without `ValueError` | Unit-tested in `test_controller_core.py` |
| **Open-Loop Goal Reached** | `closed_loop_path: false` & `dist <= goal_tolerance_m` | Smooth zero-velocity hold at terminal waypoint | Unit-tested in `test_controller_core.py` |
