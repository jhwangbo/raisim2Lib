# Joint Damping and Friction

Every revolute and prismatic joint can have viscous damping and Coulomb friction, and a spherical
joint can have viscous damping. They model bearings, seals and gears. They are passive: they only
remove energy, they are not part of the generalized force you set, and they act whether or not the
joint is controlled.

Set them with the standard URDF `<dynamics>` attributes. Both default to zero:

```xml
<joint name="knee" type="revolute">
    <parent link="thigh"/>
    <child link="shank"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.5" friction="2.0"/>
</joint>
```

| Joint type | `damping` $b$ | `friction` $\tau_c$ |
| --- | --- | --- |
| revolute | $Nm\,s/rad$ | $Nm$ |
| prismatic | $N\,s/m$ | $N$ |
| spherical | $Nm\,s/rad$, on each of the three angular velocity components | not supported (ignored) |

The MJCF loader reads the joint `damping` attribute but not `frictionloss`.

Both are measured at the joint, i.e., after any gear. If the joint is driven by an actuator, the
actuator's `output_damping` and `output_friction` add to them (see [Actuators](../Actuators.md)).

## Damping

Damping is the torque $\tau_d = -b\,u$ on the joint velocity $u$ (relative to the
parent body, see [Joints](./StateAndKinematics.md#joints)). RaiSim integrates it implicitly with the
trapezoidal rule, i.e., at the average velocity of the time step $\Delta t$:

$$
\tau_d = -b\,\frac{u_t + u_{t+1}}{2}.
$$

For example, a free joint with inertia $I$ under a constant torque $\tau$ and no gravity
accelerates as

$$
I\,\frac{u_{t+1} - u_t}{\Delta t} = \tau - b\,\frac{u_t + u_{t+1}}{2}
$$

and converges to the velocity $\tau / b$. The implicit integration keeps a joint with a large
damping stable at any time step. The d gain of the PD controller is integrated the same way. The
difference is that damping pulls the velocity to zero, while the d gain pulls it to the velocity
target.

To read the current joint damping, call `getJointDamping()`. It returns a const
`raisim::VecDyn` reference in generalized velocity order, with `getDOF()`
entries. These are the values loaded from the model or last supplied to
`setJointDamping()`; actuator output damping and PD gains are separate.
Copy `getJointDamping().e()` into an `Eigen::VectorXd` to preserve a baseline
before adding temporary damping, such as wheel braking.

To change the damping at runtime, call `setJointDamping()` with one coefficient per degree of
freedom (`getDOF()` entries). The new values act from the next step. Leave the six entries of a
floating base at zero: they are not joints, and their damping would not be integrated implicitly.

The URDF `effort` limit bounds only the commanded torque (PD plus feedforward). The damping torque
is passive and is never clipped. Actuator torques (see [Actuators](../Actuators.md)) are not bounded by the
effort limit either: they are limited by the operating regions of their motors and are added after
the effort clamp.

## Friction

Friction is Coulomb friction with stiction. It opposes the motion with the constant torque
$\tau_c$, and it holds a joint at rest as long as the other torques on it (actuation,
gravity, contacts and the motion of the other bodies) stay below $\tau_c$:

$$
\tau_f = -\tau_c\,\mathrm{sgn}(u_{t+1}) \ \text{if} \ u_{t+1} \neq 0,
\qquad |\tau_f| \le \tau_c \ \text{if} \ u_{t+1} = 0.
$$

Once the other torques exceed $\tau_c$, the joint accelerates with their sum minus
$\tau_c$. For the free joint above, that is
$I\,(u_{t+1} - u_t)/\Delta t = \tau - \tau_c\,\mathrm{sgn}(u_{t+1})$.

RaiSim solves friction as a joint impulse bounded by $\tau_c\,\Delta t$ in the contact
solver, together with the contacts and the joint limits. A joint therefore stops exactly instead
of chattering around zero velocity, and the friction torque is consistent with the contact forces
on the robot. A robot with friction adds one row to the contact
solver; robots without friction do not pay for it.

The joint's own friction is set in the model file; there is no C++ setter for it. The
`output_friction` of an actuator can be changed at runtime by editing the definitions from
`getActuators()` and passing them to `setActuators()`.
