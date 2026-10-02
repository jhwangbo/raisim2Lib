#############################
State and Kinematics
#############################

State Representation
=============================
The state of an articulated system can be represented by a **generalized state** :math:`\boldsymbol{S}`, which is composed of a **generalized coordinate** :math:`\boldsymbol{q}` and a **generalized velocity** :math:`\boldsymbol{u}`.
Since we are not constraining their parameterization, in general, 

.. math::

  \begin{equation}
    \boldsymbol{u}\neq\dot{\boldsymbol{q}}.
  \end{equation}

A generalized coordinate fully represents the configuration of the articulated system and a generalized velocity fully represents the velocity state of the articulated system.
They are independently defined in general.

Every joint has a corresponding generalized coordinate and generalized velocity.
The concatenation of all joint generalized coordinates and velocities forms the generalized coordinates and velocities of the articulated system, respectively.
The order of this concatenation is called **joint order**.
The joint order can be accessed through :code:`getMovableJointNames()`.
Note the keyword "movable".
The fixed joints contribute to neither the generalized coordinate nor the generalized velocity.
Only movable joints do.

The joint order starts with the **root body** which is the first body of the articulated system. 
For floating-base systems, the root body is the floating base.
For fixed-base systems, the root body is the one rigidly attached to the world.
Even though the fixed base cannot move physically, users can move it using :code:`setBaseOrientation` and :code:`setBasePosition`.
So :code:`getMovableJointNames()` method will return the fixed base name and the fixed base joint is a part of the joint order.

To set the state of the system, the following methods can be used

* :code:`setGeneralizedCoordinate`
* :code:`setGeneralizedVelocity`
* :code:`setState`

To obtain the state of the system, the following methods can be used

* :code:`getGeneralizedCoordinate`
* :code:`getGeneralizedVelocity`
* :code:`getState`

The dimensions of each vector can be obtained respectively by

* :code:`getGeneralizedCoordinateDim`
* :code:`getDOF` or :code:`getGeneralizedVelocityDim`. 

These two methods are identical

.. _articulated_systems:

Joints
=============================

Here are the available joints in RaiSim.

.. list-table:: Joint Properties (:math:`|\cdot|` is a symbol for dimension size (i.e., cardinality))
   :widths: 14 14 15 14 14 14
   :header-rows: 1

   * -
     - Fixed
     - Floating
     - Revolute
     - Prismatic
     - Spherical
   * - :math:`|\boldsymbol{u}|`
     - 0
     - 6
     - 1
     - 1
     - 3
   * - :math:`|\boldsymbol{q}|`
     - 0
     - 7
     - 1
     - 1
     - 4
   * - Velocity
     -
     - :math:`m/s`, :math:`rad/s`
     - :math:`rad/s`
     - :math:`m/s`
     - :math:`rad/s`
   * - Position
     -
     - :math:`m`, :math:`rad`
     - :math:`rad`
     - :math:`m`
     - :math:`rad`
   * - Force
     -
     - :math:`N`, :math:`Nm`
     - :math:`Nm`
     - :math:`N`
     - :math:`Nm`

The generalized coordinates/velocities of a joint are expressed in the **joint frame** and with respect to the **parent body**.
Joint frame is the frame attached to every joint and fixed to the parent body.
Parent body is the one closer to the root body among the two bodies connected via the joint.
Note that the angular velocity of a floating base is also expressed in the parent frame (which is the **world frame**).
Other libraries (e.g., RBDL) might have a different convention, so special care is required during conversion.

Types of Indices
=============================
The ArticulatedSystem class contains multiple types of indices. To query a specific quantity, you have to provide an index of the right type. The types of indices in Articulated Systems are:

* **Body/Joint Index**: All fixed bodies are combined to a single movable body. Each movable body has a unique body index. Because there is a movable joint associated with a movable body, there is a 1-to-1 mapping between the joints and the bodies and they share the same index. For a fixed-base system, the first body rigidly fixed to the world is body-0. For a floating-base system, the floating base is body-0.
* **Generalized Velocity (DOF) Index**: All joints are mapped to a specific set of generalized velocity indices.
* **Generalized Coordinate Index**:
* **Frame Index**:

Conversions Between Indices
*****************************
* A body index to a generalized velocity index: :code:`ArticulatedSystem::getMappingFromBodyIndexToGeneralizedVelocityIndex()`
* A body index to a generalized coordinate index: :code:`ArticulatedSystem::getMappingFromBodyIndexToGeneralizedCoordinateIndex()`

Kinematics
=============================

Frames
****************************

The position and velocity of a specific point on a body of an articulated system can be obtained by attaching a **frame**.
**Frames** are rigidly attached to a body of the system and have a constant position and orientation (w.r.t. parent frame).
This is the recommended way to get kinematics information for a point of an articulated system in RaiSim.

All joints have a frame attached and their names are the same as the joint name.
To create a custom frame, define a fixed frame at the point of interest.
A dummy link with zero inertia and zero mass must be added on one side of the fixed joint to complete the kinematic tree.

A frame can be stored locally as an index in user code. For example:

.. code-block:: cpp

  #include "raisim/World.hpp"

  int main() {
    raisim::World world;
    auto anymal = world.addArticulatedSystem(PATH_TO_URDF);
    auto footFrameIndex = anymal->getFrameIdxByName("foot_joint"); // the URDF has a joint named "foot_joint"
    raisim::Vec<3> footPosition, footVelocity, footAngularVelocity;
    raisim::Mat<3,3> footOrientation;
    anymal->getFramePosition(footFrameIndex, footPosition);
    anymal->getFrameOrientation(footFrameIndex, footOrientation);
    anymal->getFrameVelocity(footFrameIndex, footVelocity);
    anymal->getFrameAngularVelocity(footFrameIndex, footAngularVelocity);
  }

You can also store a Frame reference.
For example, you can replace :code:`getFrameIdxByName` with :code:`getFrameByName` in the example above.
In this way, you can access internal variables and modify them.
Modifying frames does not affect the joints.
Frames are instantiated during initialization of the articulated system instance and affect neither kinematics nor dynamics, even if you change them.

Joint limits
************************
Joint limits can be defined in a URDF file **per joint** as follows:

.. code-block:: xml

   <limit effort="80" lower="-6.28" upper="6.28" velocity="15"/>

The ``lower`` and ``upper`` are joint position limits and the ``velocity`` is the joint velocity limit.
The joint limits are implemented as if there is a hard stop at the limits.
This means that there is a hard collision (with a restitution coefficient of 0) when the joint hits a limit.

You can modify the joint position limits in C++ using ``raisim::ArticulatedSystem::setJointLimits()``.
Currently, you cannot modify the velocity joint limits in code.

During simulation, you can get information on joint limit violations using ``raisim::ArticulatedSystem::getJointLimitViolations``.
Even though joint limits are collisions (and thus handled by a contact solver), they are not listed in ``raisim::Object::getContacts()``.

Jacobians
****************************
Jacobians of a point in RaiSim satisfy the following equation:

.. math::

  \begin{equation}
    \boldsymbol{J}\boldsymbol{u} = \boldsymbol{v}
  \end{equation}

where :math:`\boldsymbol{v}` represents the linear velocity of the associated point.
If a rotational Jacobian is used, the right-hand side changes to a rotational velocity expressed in the world frame.

To get the Jacobians associated with the linear velocity, the following methods are used

* :code:`getSparseJacobian`
* :code:`getDenseJacobian` -- this method only fills non-zero values. The matrix should be initialized to a zero matrix of an appropriate size.

To get the rotational Jacobians, the following methods are used

* :code:`getSparseRotationalJacobian`
* :code:`getDenseRotationalJacobian` -- this method only fills non-zero values. The matrix should be initialized to a zero matrix of an appropriate size.

The main Jacobian class in RaiSim is :code:`raisim::SparseJacobian`. 
RaiSim uses only sparse Jacobians because it is more memory-efficient.
Note that only the joints between the child body and the root body affect the motion of the point.

The class :code:`raisim::SparseJacobian` has a member :code:`idx` which stores the indices of columns whose values are non-zero.
The member :code:`v` stores the Jacobian except the zero columns.
In other words, ith column of :code:`v` corresponds to :code:`idx[i]` generalized velocity dimension.
