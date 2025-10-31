# mobility control

## 1. Pure Pursuit Algorithm

**Pure Pursuit** is a geometric path-tracking controller that steers a robot/vehicle to follow a given reference path using the vehicle’s kinematics and the geometric properties of the path.  
> In the standard formulation, speed is assumed fixed; the controller computes **only the steering angle**.

<img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/665420a5-7f96-4929-a712-048a054bdcf4" />

In the figure above, the blue curve is the **trajectory (reference path)**. The controller selects **look-ahead points** on the trajectory at a fixed distance from the vehicle’s **rear axle**. This distance is the **look-ahead distance**, denoted by \(\ell_d\).

<img width="500" height="400" alt="image" src="https://github.com/user-attachments/assets/7d1db55e-b34c-49a6-8715-2df6a5af7033" />

- Waypoints & trajectory are outputs from the planner.  
- The actual driven path can deviate from the waypoints.  
- Pure Pursuit computes the steering command that drives the vehicle toward a selected waypoint/look-ahead point.

<img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/01c9ff9b-6fb1-403a-8923-11f6d2a6ffea" />

The method uses a simple **bicycle model** (two-wheel kinematics). Turning the steering angle \(\delta\) makes the vehicle follow a **circular arc** of **radius** \(R\).  
Let \(\alpha\) denote the angle between the vehicle’s heading and the straight line from the rear axle to the look-ahead point (the “look-ahead line”). Because the vehicle’s instantaneous motion is tangent to the circle, the angle at the circle center subtended by the chord (rear axle to look-ahead point) is \(2\alpha\).

---

### Geometric Relations and Curvature

From the circle geometry (law of sines),

$$
\frac{\ell_d}{\sin(2\alpha)} \=\ \frac{R}{\sin\left(\tfrac{\pi}{2}-\alpha\right)}.
$$

Using $$\(\sin(2\alpha)=2\sin\alpha\cos\alpha\)$$ and $$\(\sin(\tfrac{\pi}{2}-\alpha)=\cos\alpha\)$$,

$$
\frac{\ell_d}{\sin\alpha\,\cos\alpha} \=\ \frac{R}{\cos\alpha}.
$$

Since curvature is the inverse of radius, we obtain

$$
k \=\ \frac{1}{R} \=\ \frac{2\,\sin\alpha}{\ell_d}.
$$

The steering–radius relation from the bicycle model is

$$
R \=\ \frac{L}{\tan\delta},
$$

hence the steering angle that reaches the target point is

$$
\delta \=\ \arctan\Big(\frac{2L\,\sin\alpha}{\ell_d}\Big).
$$

---

### Kinematic Bicycle Model

The vehicle’s longitudinal/lateral velocities and yaw rate are

$$
\dot{x} \=\ v\cos\theta,\qquad
\dot{y} \=\ v\sin\theta,\qquad
\dot{\theta} \=\ \omega \=\ \frac{v}{R}
$$

Equivalently, using the steering relation,

$$
\dot{\theta} \=\ \frac{v\tan\delta}{L}
$$

---

## How to set the look-ahead distance?

<img width="600" height="600" alt="image" src="https://github.com/user-attachments/assets/0cc32bcc-aed7-4fba-8751-ab26db54f6b9" />

- **Short $$\(\ell_d\)$$**: faster convergence but less stable; the path may oscillate while negotiating sharp turns.  
- **Long $$\(\ell_d\)$$**: slower convergence but more stable; the path may turn early and “cut” corners on sharp trajectories.

<img width="354" height="230" alt="image" src="https://github.com/user-attachments/assets/90fc37a5-6bb7-48d7-9250-394abc5c235f" />

Because curvature grows with cross-track error $$\(e\)$$, larger errors yield stronger corrective curvature:

$$
\sin\alpha \=\ \frac{e}{\ell_d},
\qquad
k \=\ \frac{2\sin\alpha}{\ell_d},
\qquad
k \=\ \frac{2}{\ell_d^{\,2}}e.
$$

## 2. Stanley Controller

The **Stanley controller** computes a steering command that drives a vehicle to follow a reference path by regulating (i) the **heading error** with respect to the path tangent and (ii) the **cross-track error** (lateral offset) to the path.

<img width="400" height="400" alt="image" src="https://github.com/user-attachments/assets/69dd06e6-0da9-461e-824c-cd6ef2215bc2" />

### Notation
- $$\( (x, y, \psi) \)$$: vehicle rear-axle pose (world frame), heading $$\( \psi \)$$.
- $$\( \psi_p \)$$: path heading (tangent angle) at the closest point on the path.
- $$\( e \)$$: signed cross-track error (lateral distance from rear axle to the path, left $$\(+\)$$, right $$\(-\))$$.
- $$\( v \)$$: vehicle speed.
- $$\( L \)$$: wheelbase, $$\( \delta \)$$: steering angle.

### Heading error
The **heading error** is

$$
\theta_e = \mathrm{atan2}\big(\sin(\psi_p - \psi),\, \cos(\psi_p - \psi)\big)
$$

### Stanley steering law
The steering command combines heading regulation and lateral regulation:

$$
\delta \=\ \theta_e \+\ \arctan\Big(\frac{k\,e}{v + \varepsilon}\Big),
$$

where $$\(k>0\)$$ is the cross-track gain and $$\(\varepsilon\ge 0\)$$ is a small constant to avoid division by zero and to temper the control at very low speeds (e.g., $$\(\varepsilon \approx 0.1\sim0.5\)$$).

> **Sign convention:** define $$\(e>0\)$$ when the path lies to the **left** of the vehicle’s longitudinal axis. Ensure the closest-point projection and the path tangent $$\( \psi_p \)$$ are computed consistently with this convention.

### Kinematics (bicycle model)
For completeness, the yaw-rate relation is

$$
\dot{\psi} \=\ \frac{v}{L}\tan\delta,
$$

and the planar kinematics are

$$
\dot{x} \=\ v\cos\psi,\qquad
\dot{y} \=\ v\sin\psi.
$$

### Practical notes
- **Saturation:** clip $$\( \delta \)$$ to hardware limits $$\( |\delta| \le \delta_{\max} \)$$.
- **Gain tuning:** larger $$\(k\)$$ reduces steady cross-track error but may induce oscillation; use a modest $$\(k\)$$ and nonzero $$\(\varepsilon\)$$ for low-speed robustness.
- **Projection:** compute the closest point on the reference path to obtain $$\(e\)$$ and $$\( \psi_p \)$$ update every control cycle.



