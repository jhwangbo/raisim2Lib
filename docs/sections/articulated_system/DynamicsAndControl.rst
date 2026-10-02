#############################
Dynamics and Control
#############################

Dynamics
=============================
All force and torque acting on the system can be represented as a single vector in the generalized velocity space.
This representation is called **generalized force** :math:`\boldsymbol{\tau}`.
Just like in a Cartesian coordinate (i.e., x, y, z axes), the power exerted by an articulated system is computed as a dot product of generalized force and generalized velocity (i.e., :math:`\boldsymbol{u}\cdot\boldsymbol{\tau}`).

We can also combine the mass and inertia of the whole articulated system and represent them in a single matrix.
This matrix is called **mass matrix** or **inertia matrix** and denoted by :math:`\boldsymbol{M}`. 
A mass matrix represents how much the articulated system resists change in generalized velocities.
Naively speaking, a large mass matrix means that the articulated system experiences a low velocity change for a given generalized force.

The total kinetic energy of the system is computed as :math:`\frac{1}{2}\boldsymbol{u}^T\boldsymbol{M}\boldsymbol{u}`.
This quantity can be obtained by :code:`getKineticEnergy()`.

The total potential energy due to the gravity is a sum of :math:`mgh` for all bodies.
This quantity can be obtained by :code:`getPotentialEnergy()`.
Note that gravity must be specified since only the world stores the gravity vector.

The equation of motion of an articulated system is shown below:

.. math::

  \begin{equation}
     \boldsymbol{\tau} = \boldsymbol{M}(\boldsymbol{q})\dot{\boldsymbol{u}} + \boldsymbol{h}(\boldsymbol{q}, \boldsymbol{u}).
  \end{equation}

Here :math:`\boldsymbol{h}` is called a **non-linear term**. 
There are three sources of force that contribute to the non-linear term: gravity, Coriolis, and centrifugal force.
It is rarely useful to compute the gravity contribution to the nonlinear term alone.
However, if it is needed, the most direct way is to make the same robot in another world with zero velocity.
If the generalized velocity is zero, the coriolis and centrifugal contributions are zero.

The following methods are used to obtain dynamic quantities

* :code:`getMassMatrix()`
* :code:`getNonlinearities()`
* :code:`getInverseMassMatrix()`

Inverse Dynamics
==================
RaiSim can compute inverse dynamics using the recursive Newton-Euler algorithm.
It is the only option for computing the force and torque acting at joints.
Joint force/torque are the sum of the constraint joint force/torque and actuation force/torque.
For example, a revolute joint constrains motions in 5 degrees of freedom, which means that there are 5-dimensional constraint forces/torque and 1-dimensional joint actuation torque acting at a revolute joint.

In minimal coordinate simulation (such as RaiSim), these constraint forces/torques are not computed in the simulation loop.
These forces/torques can be computed after a simulation loop using the inverse dynamics pipeline.

To enable inverse dynamics, call ``raisim::ArticulatedSystem::setComputeInverseDynamics(true)``.
**This flag is set automatically if the robot has an IMU sensor**.
Note that the inverse dynamics pipeline will slow down the simulation by about 10\%.

After a simulation loop, you can call ``raisim::ArticulatedSystem::getForceAtJointInWorldFrame()`` and ``raisim::ArticulatedSystem::getTorqueAtJointInWorldFrame()`` to get forces and torques acting at the specified joint.

Assuming that there are no joint position/velocity limit forces acting at the joint, you can compute the joint actuation as a dot product of the joint axis and the joint torque.
The current articulated examples, such as ``anymal_pair`` and ``atlas``, show
the public example workflow for loading and stepping articulated systems.

PD Controller
=============================
When naively implemented, a PD controller can often make a robot unstable.
However, this is often less problematic for robotics since this instability is also present in real systems (discrete-time control systems).

For other applications like animation and graphics, it is often desirable to have a stable PD controller when a user wants to keep the time step small.
Therefore, this PD controller exploits a more stable integration scheme and can use a much smaller time step than a naive implementation.

**This PD controller does not respect the actuation limits of the robot**.
It uses an implicit integration scheme and we do not even compute the actual torque that is applied to the joints.

To use this PD controller, set the desired control gains first:

.. code-block:: cpp

  Eigen::VectorXd pGain(robot->getDOF()), dGain(robot->getDOF());
  pGain<< ...; // set your proportional gain values here
  dGain<< ...; // set your differential gain values here
  robot->setPdGains(pGain, dGain);

Note that **the dimension of the pGain vector is the same as that of the generalized velocity NOT that of the coordinate**.

Finally, the target position and the velocity can be set as follows:

.. code-block:: cpp

  Eigen::VectorXd pTarget(robot->getGeneralizedCoordinateDim()), vTarget(robot->getDOF());
  pTarget<< ...; // set your position target
  vTarget<< ...; // set your velocity target
  robot->setPdTarget(pTarget, vTarget);

Here, **the dimension of the pTarget vector is the same as that of the generalized coordinate NOT that of the velocity**.
This can be confusing and may seem inconsistent.
However, this is a valid convention.
The only reason the two dimensions differ is the quaternion representation.
The quaternion target is represented by a quaternion whereas the virtual spring stiffness between the two orientations can be represented by a 3D vector, which is composed of motions in each angular velocity components.

A feedforward force term can be added by :code:`setGeneralizedForce()` if desired.
This term is set to zero by default.
Note that this value is stored in the class instance and does not change unless the user specifies it so.
If this feedforward force should be applied for a single time step, set it to zero in the subsequent control loop (after the :code:`integrate()` call of the world).

The theory of the implemented PD controller can be found in chapter 1.2 of this `article <https://www.overleaf.com/read/dbqbgcnhzykq>`_.
This document is intended for advanced users and is not required to use RaiSim.

Actuators
=============================
The URDF ``effort`` limit is a constant bound per joint. To limit the joint torques to what the
motors can produce at their speed (bus voltage, back EMF and peak torque, including coupled
transmissions), attach actuators to the joints with ``<actuator>`` elements. See :doc:`../Actuators`.

Apply External Forces/Torques
=============================
The following two methods are used to apply external force and torque respectively

* :code:`setExternalForce`
* :code:`setExternalTorque`

You will find the above methods in the :doc:`API`.

Integration Steps
=============================
Integration of an articulated system is performed in two stages: :code:`integrate1` and :code:`integrate2`.

The following steps are performed in :code:`integrate1`

1. If the time step is changed, update the damping of the mass matrix (which reflects effective inertial increase due to springs, dampers and PD gains)
2. Update positions of the collision bodies
3. Detect collisions (called by the world instance)
4. The world assigns contacts on each object and computes the contact normal
5. Compute the mass matrix, nonlinear term, and inverse inertia matrix
6. Compute (Sparse) Jacobians of contacts

After this step, all kinematic/dynamic properties are computed.
Users can access them if they are necessary for the controller.
Next, :code:`integrate2` computes the rest of the simulation.

7. Compute contact properties
8. Compute PD controller (if used), add it to the feedforward force and bound it by the limits
9. Compute generalized forces due to joint damping (see :doc:`JointDampingAndFriction`), springs and external forces/torques
10. Contact solver (called by the world instance). It also solves the joint friction and the motor operating regions
11. Integrate the velocity
12. Integrate the position (in a semi-implicit way)
