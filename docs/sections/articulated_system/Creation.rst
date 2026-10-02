###############################
Creating an Articulated System
###############################

Creating an instance
=============================
Like any other object, an articulated system is created by the world instance using the :code:`addArticulatedSystem` method.
There are four ways to specify an articulated system.

1. by providing the path to the URDF file (recommended)
2. by providing the path to the URDF template file
3. by providing a :code:`std::string` containing the URDF text (useful when working with Xacro)
4. by providing a :code:`raisim::Child` instance (advanced; not recommended for beginners)

**Note that option 1 and 3 use the same method.**
**You can provide either the path string or the contents string and the class will identify which one is provided.**

To use option 4, you have to provide all details of the robot in C++ code.
Child is a tree node that contains ``fixedBodies`` and ``child``.
It also contains ``joint``, ``body``, and ``name``.
All properties should be filled.
Make sure that all inertial properties are defined so that the resulting system is physically feasible.

URDF convention
=============================
RaiSim uses a modified URDF protocol to define an articulated system.
URDF files following the original convention can be read by RaiSim.
However, since RaiSim offers more features, a RaiSim URDF might not be read by other libraries following the original URDF convention.

The modifications are as follows:

* Capsule geometry is available for both collision objects and visual objects (with the keyword "capsule"). The geometry is defined by the "height" and "radius" keywords. The height represents the distance between the centers of the two spheres.

* A <joint>/<dynamics> tag can have three more attributes: *rotor_inertia*, *spring_mount* and *stiffness*.
  Its standard *damping* and *friction* attributes are described in :doc:`JointDampingAndFriction`.

* A <robot> can have <actuator> tags. They attach motors and transmissions to joints and limit the joint torques to the motor operating regions (see :doc:`../Actuators`).

Here is an example joint with the RaiSim tags:

.. code-block:: xml

    <joint name="link1Tolink2" type="spherical">
        <parent link="link1"/>
        <child link="link2"/>
        <origin xyz="0 0 -0.24"/>
        <axis xyz="0 1 0"/>
        <dynamics rotor_inertia="0.0001" spring_mount="0.70710678118 0 0.70710678118 0" stiffness="500.0" damping="3."/>
    </joint>

**Rotor_inertia** in RaiSim approximately simulates the rotor inertia of the motor (but omits the resulting gyroscopic effect, which is often negligible).

It is added to the diagonal elements of the mass matrix.
It is a common way to include the inertial effect of the rotor.
You can also override it in C++ using :code:`setRotorInertia()`.
Since RaiSim does not know the gear ratio, you have to multiply the rotor inertia by the square of the gear ratio yourself.
In other words, the value is the reflected rotor inertia observed at the joint.

Two preprocessor features (also available in the RaiSim world configuration file) are available for the URDF template.

* You can specify a variable in a form of "@@Robot_Height". The value of this variable can be specified at runtime using ``std::unordered_map`` and the corresponding factory method in ``raisim::World``.

* You can specify an equation instead of a variable. For example, {@@Robot_Height*@@Robot_Width*2}.

The corresponding URDF template format is shown by resources such as
``rsc/templatedTrackedRobot/trackedTemplate.urdf``.


In RaiSim, each body of an articulated system has a set of collision bodies and visual objects.
Collision bodies contain a collision object of one of the following shapes: *mesh*, *sphere*, *box*, *cylinder*, *capsule*.
Visual objects store specifications for visualization; the actual visualization happens in a visualizer.
For details, check the `URDF protocol <http://wiki.ros.org/urdf/XML>`_.

Mesh collision mode for URDF
*******************************
URDF does not provide a tag to choose the mesh collision mode. In RaiSim, the mode
is selected at load time with ``ArticulatedSystemOption::convexifyCollisionMeshes``.
The default is ``false``, which keeps each URDF collision mesh as the original
non-convex triangle mesh. Set it to ``true`` to build a convex contact mesh for
each URDF mesh collision element:

.. code-block:: cpp

  raisim::ArticulatedSystemOption options;
  options.convexifyCollisionMeshes = true;
  auto* robot = world.addArticulatedSystem(urdfPath, "", {}, 1, -1, options);

This articulated-system option is a simple original-mesh versus convex-mesh
choice. It does not expose the single-body ``CoacdOptions`` path. Use authored
simplified collision meshes in the URDF when you need precise control over
robot-link collision geometry. Use ``raisim::World::addMesh`` with
``MeshCollisionMode::CONVEXIFY`` for standalone mesh objects that should use
CoACD convex decomposition:

.. code-block:: cpp

  raisim::CoacdOptions coacd;
  coacd.threshold = 0.06;
  coacd.maxConvexHull = 12;

  auto* prop = world.addMesh(meshPath, mass, scale, "default",
                             raisim::MeshCollisionMode::CONVEXIFY,
                             raisim::CollisionGroup(1),
                             raisim::CollisionGroup(-1),
                             coacd);

For articulated robots, original non-convex triangle-mesh collision can be
slower and less robust than primitive, convex, or authored collision geometry.
Prefer primitive collision shapes or simplified convex mesh assets for feet,
hands, wheels, and links that touch terrain frequently.

Templated URDF
*******************************
You can template a URDF and create different robots by providing different parameters in C++.
An example can be found `here <https://github.com/raisimTech/raisim2Lib/tree/master/rsc/templatedTrackedRobot>`__.

In the URDF template, variables should be marked with ``@@``.
Just like in a world configuration template, you can write math expressions inside ``{}``.
Only basic functions (i.e., sin, cos, log, exp) are available.

Template parameters should be provided at runtime in ``raisim::World::addArticulatedSystem``.
One of the overloaded methods takes ``const std::unordered_map<std::string, std::string>& params`` as input.
The first one in the pair is the name and the second one is the parameter as a string.

URDF modules (optional attachments)
====================================
Articulated systems can be created from a base URDF plus one or more *modules*.
A module is a URDF fragment (e.g., additional links/joints/sensors) that is
inserted into the base URDF before the closing ``</robot>`` tag. This is useful
for optional payloads or sensor packs without duplicating the entire URDF.

Use the constructor that takes a list of module filenames:

.. code-block:: cpp

  std::vector<std::string> modules = {"d455.xml", "livox.xml"};
  auto* robot = world.addArticulatedSystem(urdfPath, modules, resDir);

Module path resolution follows this order (first match wins):

* absolute path to the module file
* ``[baseDir]/<module>``
* ``[baseDir]/modules/<module>``
* ``[baseDir]/../modules/<module>``
* ``[baseDir]/module/<module>``
* ``[baseDir]/../module/<module>``

``baseDir`` is the URDF directory unless you pass ``resDir`` explicitly.
RaiSim writes a generated URDF named
``generated_<urdf_stem>_<module_stem>[_<module_stem>...].urdf`` into ``baseDir``
and loads that file.

URDF loader details
=============================
The URDF loader used by :code:`ArticulatedSystem` has a few behaviors worth noting:

* **Base type:** If the root link is named ``world``, RaiSim treats the system as
  fixed-base; otherwise it is floating-base.
* **Sensors:** ``<link sensor="...">`` loads a sensor set XML (see :doc:`../Sensors`) and instantiates
  sensors of type ``rgb``, ``depth``, ``imu``, or ``spinning_lidar``. The
  ``update_rate`` attribute in the sensor XML is applied to each sensor. IMU
  sensors enable inverse dynamics internally.
* **Constraints:** A ``<constraints>`` block with ``<pin>`` entries and a
  ``nominal_config`` attribute is parsed and passed to the constraint system.
