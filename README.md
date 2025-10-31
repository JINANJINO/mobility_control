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
\frac{\ell_d}{\sin(2\alpha)} \=\ \frac{R}{\sin\left(\tfrac{\pi}{2}-\alpha\right)} \, .
$$

Using $$\(\sin(2\alpha)=2\sin\alpha\cos\alpha\)$$ and $$\(\sin(\tfrac{\pi}{2}-\alpha)=\cos\alpha\)$$,

$$
\frac{\ell_d}{\sin\alpha\,\cos\alpha} \=\ \frac{R}{\cos\alpha} \, .
$$

Since curvature is the inverse of radius, we obtain

$$
k \=\ \frac{1}{R} \=\ \frac{2\,\sin\alpha}{\ell_d} \, .
$$

The steering–radius relation from the bicycle model is

$$
R \=\ \frac{L}{\tan\delta} \, ,
$$

hence the steering angle that reaches the target point is

$$
\delta \=\ \arctan\Big(\frac{2L\,\sin\alpha}{\ell_d}\Big) \, .
$$

---

### Kinematic Bicycle Model

The vehicle’s longitudinal/lateral velocities and yaw rate are

$$
\dot{x} \=\ v\cos\theta,\qquad
\dot{y} \=\ v\sin\theta,\qquad
\dot{\theta} \=\ \omega \;=\; \frac{v}{R} \, .
$$

Equivalently, using the steering relation,

$$
\dot{\theta} \=\ \frac{v\,\tan\delta}{L} \, .
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
k \=\ \frac{2\,e}{\ell_d^{\,2}} \, .
$$
