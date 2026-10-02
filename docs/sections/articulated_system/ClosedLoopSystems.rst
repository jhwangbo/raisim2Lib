#############################
Closed-Loop Systems
#############################

Before modeling a closed-loop system, it is necessary to model a corresponding spanning tree.
A spanning tree is a kinematic tree that can be constructed by removing a minimum number of joints from a closed-loop system.
Imagine a chain necklace. By disconnecting one of the joints, a kinematic tree will form.
Only one joint should be removed because, otherwise, two separate kinematic trees will form.
Note that there are multiple ways to form a kinematic tree because any of the joints can be removed.

To model a closed-loop system in RaiSim, a corresponding spanning tree should be modeled in a URDF format first.
To convert the spanning tree into a closed-loop system, add pin constraints in the URDF.
A pin constraint is an equality constraint that enforces two points on different bodies to occupy the same world position.
Each pin is defined by:

* ``body1`` and ``body2``: link names to be constrained.
* ``anchor``: the point in the *link frame of ``body1``*.
* ``nominal_config``: a full generalized coordinate vector used once at
  initialization to compute the matching anchor on ``body2``.

``nominal_config`` must match the generalized coordinate order used by
:code:`getGeneralizedCoordinate()` (equivalently, the joint order returned by
:code:`getMovableJointNames()`). For floating-base systems, include the base
position and base quaternion first, followed by joint coordinates.

Example (minitaur) constraints block:

.. code-block:: xml

    <constraints nominal_config="0 0 0.35 0 0 1 0 -1.5708 -2.2 -1.5708 -2.2 -1.5708 -2.2 -1.5708 -2.2 -1.5708 -2.2 -1.5708 -2.2 -1.5708 -2.2 -1.5708 -2.2">
        <pin body1="lower_leg_front_rightR_link" body2="lower_leg_front_rightL_link" anchor="0.0 0.0 0.2"/>
        <pin body1="lower_leg_front_leftR_link" body2="lower_leg_front_leftL_link" anchor="0.0 0.0 0.2"/>
        <pin body1="lower_leg_back_rightR_link" body2="lower_leg_back_rightL_link" anchor="0.0 0.0 0.2"/>
        <pin body1="lower_leg_back_leftR_link" body2="lower_leg_back_leftL_link" anchor="0.0 0.0 0.2"/>
    </constraints>

At initialization, RaiSim sets the system to ``nominal_config``, computes the
world position of ``anchor`` on ``body1``, and derives the corresponding anchor
on ``body2``. After that, the constraint is enforced every step by the contact
solver.

Contact solver handling
************************
Closed-loop pin constraints are injected into the same contact solver that
handles collisions. Internally, pin constraints are treated as special
contact problems (rank-1/2/3 pin constraints) and are solved with impulse
updates each iteration. A small ERP term corrects residual position error
at every step. The contact solver itself uses a bisection-based method for
frictional contacts; pin constraints bypass friction handling and are solved
as equality constraints inside the same loop. This makes closed-loop systems
numerically robust in RaiSim, because the loop constraints are enforced
through the same stabilized contact solver that resolves impacts and friction.

An example of a closed-loop system URDF can be found `here <https://github.com/raisimTech/raisim2Lib/blob/master/rsc/minitaur/minitaur.urdf>`__.
