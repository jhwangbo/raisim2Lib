#############################
Modifying the Model in Code
#############################

RaiSim allows users to modify most of the robot parameters freely in code.
This allows users to create randomized robot models, which might be useful for AI applications (i.e., **dynamic randomization**).
Note that a random model might be kinematically and dynamically unrealistic.
For example, joints can be locked by collision bodies.
In such cases, simulation cannot be performed reliably and it is advised to carefully check randomly generated robot models.

Here is a list of modifiable kinematic/dynamic parameters.

* **Joint Position (relative to the parent joint) Expressed in the Parent Frame**

:code:`getJointPos_P` method returns (a non-const reference to) a :code:`std::vector` of position vectors from the parent joint to the child joint expressed in the respective parent joint frames.
This should be changed with care since it can result in unrealistic collision geometry.
**This method does not change the position of the end-effector with respect to its parent** as the position of the last link is defined by the collision body position, not by the joint position.
The elements are ordered by the joint indices.

* **Joint Axis in the Parent Frame**

:code:`getJointAxis_P` method returns (a non-const reference to) a :code:`std::vector` of joint axes expressed in the respective parent joint frame.
This method should also be changed with care.
The elements are ordered by the joint indices.

* **Mass of the Links**

:code:`getMass` method returns (a non-const reference to) a :code:`std::vector` of link masses.
**IMPORTANT! You must call :code:`updateMassInfo`** after changing mass values.
The elements are ordered by the body indices (which is the same as the joint indices in RaiSim).

* **Center of Mass Position**

:code:`getBodyCOM_B` method returns (a non-const reference to) a :code:`std::vector` of the COM of the bodies.
The elements are ordered by the body indices.

* **Link Inertia**

:code:`getInertia` method returns (a non-const reference to) a :code:`std::vector` of link inertia.
The elements are ordered by the body indices.

* **Collision Bodies**

:code:`getCollisionBodies` method returns (a non-const reference to) a :code:`std::vector` of the collision bodies.
This vector contains all collision bodies associated with the articulated system.

:code:`getCollisionBody` method returns a specific collision body instead.
All collision bodies are named "LINK_NAME" + "/INDEX". 
For example, the 2nd collision body of a link named "FOOT" is named "FOOT/1" (1 because the index starts from 0).

The collision body is a class that contains the position/orientation offset from the parent joint frame, name, parent body index, and the ODE collision pointer (:code:`dGeomID`, retrieved using :code:`getCollisionObject`).
The collision geom can be modified using ODE methods (`ODE manual <http://ode.org/wiki/index.php?title=Manual>`_).
Users can also modify the material of the collision body.
This material affects the contact dynamics.

Collision
==============================
Apart from the collision mask and collision group set in the world, users can also disable a collision between a certain pair of links with :code:`ignoreCollisionBetween`.
