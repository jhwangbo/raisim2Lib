"""Suspension geometry, mass properties and URDF of the rayrai off-road buggy.

The buggy is an articulated system whose closed loops are described in the
URDF's <constraints> element:

* Each corner is a double wishbone. The lower arm hinges on the chassis and
  carries the upright on a spherical joint (the lower ball joint); the upper
  arm hinges on the chassis and is pinned to the upright at the upper ball
  joint.
* A tie rod hangs from the steering rack (front) or the chassis (rear toe
  link) on a spherical joint and is pinned to the upright's steering arm.
* The coilover is a damper body hinged to the chassis and a rod sliding in it
  on a prismatic joint with the coil spring and the damper
  (<dynamics stiffness spring_mount damping>). The rod eye is tied to the
  lower arm by an equality constraint along the two axes of the arm's plane.
* The rack follows the steering wheel through a mimic constraint (rack and
  pinion).
* The wheel rides on two sprung, damped prismatic joints for the radial and
  lateral compliance of the tyre. A rigid tyre could not scrub sideways as the
  wishbones swing, so friction would lock the suspension.

Frames: the chassis frame has x forward, y left and z up, with its origin
midway between the axles at wheel-centre height. Every link frame starts
aligned with the chassis frame, except the two coilover links, whose z axis
runs along the damper from the rod eye to the top eye.
"""
import math

import numpy as np

G = 9.81
# Axle positions and tyre widths measured from the Sketchfab model at its scale
# (prepare_offroad_buggy_assets.py, SCALE); the wheel centre is at z = 0.
AXLES = {
    'f': dict(x=1.3571, track=1.1108, width=0.229, steer=True),
    'r': dict(x=-1.3571, track=1.0840, width=0.284, steer=False),
}
WHEEL_RADIUS = 0.3611
# Left-side hardpoints. Ball joints are relative to the wheel centre; inboard
# pivots have absolute y and z.
LBJ = np.array([0.0, -0.085, -0.16])
UBJ = np.array([-0.015, -0.115, 0.16])
LOWER_INNER = dict(y=0.50, z=-0.13, spread=0.16)  # spread: half distance of the two arm bushings
UPPER_INNER = dict(y=0.62, z=0.15, spread=0.13)
TIE_X = -0.13   # steering arms and toe links lie this far behind the axle
TIE_Z = -0.02
SHOCK_LOWER = dict(y=0.86, z=-0.11)   # rod eye on the lower arm
SHOCK_UPPER = dict(y=0.70, z=0.27)    # top eye on the chassis
# Damper stroke from the design length: compression (bump stop) and extension.
SHOCK_TRAVEL = (-0.070, 0.060)
RACK_TRAVEL = 0.055
ACKERMANN_AIM = -0.75
PINION_RADIUS = 0.0095  # rack travel per steering-wheel radian [m]

MASS = dict(chassis=730.0, lower_arm=4.0, upper_arm=2.5, upright=6.0, wheel=24.0, tie_rod=0.8,
            shock_body=2.0, shock_rod=0.8, rack=2.5, steering_wheel=1.5)
# Sprung body (with driver and drive unit): centre of mass and principal inertia.
CHASSIS_COM = np.array([-0.14, 0.0, 0.10])
CHASSIS_INERTIA = (230.0, 800.0, 900.0)
WHEEL_INERTIA = (0.75, 1.25, 0.75)
# Target heave frequency of the sprung mass on each axle and damping ratio.
RIDE_FREQUENCY = dict(f=1.45, r=1.55)
DAMPING_RATIO = dict(f=0.38, r=0.38)

CORNERS = [('fl', 'f', 1), ('fr', 'f', -1), ('rl', 'r', 1), ('rr', 'r', -1)]
# Tyre: radial and lateral stiffness and damping of a 28-inch off-road tyre at low pressure, travel
# of the compliance joints, and the mass of the two massless-in-reality carriers.
TYRE = dict(radial_stiffness=180e3, radial_damping=500.0, radial_travel=(-0.03, 0.05),
            lateral_stiffness=110e3, lateral_damping=600.0, lateral_travel=0.025, contact_width=0.55,
            carrier_mass=0.2)


def corner_points(axle, side):
    """Hardpoints of one corner in the chassis frame; side is +1 (left) or -1 (right)."""
    a = AXLES[axle]
    wheel = np.array([a['x'], a['track'], 0.0])
    lbj, ubj = wheel + LBJ, wheel + UBJ
    p = dict(wheel=wheel, lbj=lbj, ubj=ubj,
             lower_inner=np.array([a['x'], LOWER_INNER['y'], LOWER_INNER['z']]),
             upper_inner=np.array([a['x'] + UBJ[0], UPPER_INNER['y'], UPPER_INNER['z']]))
    # The kingpin axis runs through both ball joints; take its point at tie-rod height.
    king = lbj + (TIE_Z - lbj[2]) / (ubj[2] - lbj[2]) * (ubj - lbj)
    x_tie = a['x'] + TIE_X
    if a['steer']:
        # Ackermann: the line from the kingpin through the steering arm's joint points at
        # ACKERMANN_AIM on the centre line (the rear axle in the classic rule; ahead of it here,
        # because the tie rods sit behind the axle and the rack is short).
        d = np.array([ACKERMANN_AIM, 0.0, TIE_Z]) - king
        otr = king + d * ((x_tie - king[0]) / d[0])
    else:
        otr = np.array([x_tie, king[1], TIE_Z])
    # In the front view the inner tie-rod joint lies on the line through the
    # arms' inner pivots, which keeps bump steer small.
    li, ui = p['lower_inner'], p['upper_inner']
    s = (TIE_Z - li[2]) / (ui[2] - li[2])
    itr = np.array([x_tie, li[1] + s * (ui[1] - li[1]), TIE_Z])
    p.update(king=king, otr=otr, itr=itr,
             shock_lower=np.array([a['x'], SHOCK_LOWER['y'], SHOCK_LOWER['z']]),
             shock_upper=np.array([a['x'], SHOCK_UPPER['y'], SHOCK_UPPER['z']]))
    mirror = np.array([1.0, side, 1.0])
    return {k: v * mirror for k, v in p.items()}


def shock_angle(p):
    """Rotation about x that turns the chassis z axis onto the damper axis (rod eye to top eye)."""
    axis = p['shock_upper'] - p['shock_lower']
    return math.atan2(-axis[1], axis[2])


def rack_origin():
    p = corner_points('f', 1)
    return np.array([p['itr'][0], 0.0, p['itr'][2]])


def front_view(p, alpha):
    """The double wishbone in the front view (y, z) with the lower arm turned by alpha about x:
    returns the tyre contact point and the damper length. The upper ball joint follows from the
    four-bar linkage lower arm, upright, upper arm, chassis."""
    def rot(v, a):
        return np.array([math.cos(a) * v[0] - math.sin(a) * v[1], math.sin(a) * v[0] + math.cos(a) * v[1]])
    li, ui, lbj0, ubj0 = (p[k][1:] for k in ('lower_inner', 'upper_inner', 'lbj', 'ubj'))
    lbj = li + rot(lbj0 - li, alpha)
    upper, coupler = np.linalg.norm(ubj0 - ui), np.linalg.norm(ubj0 - lbj0)
    # Intersection of the circles about the upper pivot and the lower ball joint near the design pose.
    d = np.linalg.norm(lbj - ui)
    a = (upper ** 2 - coupler ** 2 + d ** 2) / (2 * d)
    h = math.sqrt(max(upper ** 2 - a ** 2, 0.0))
    base = ui + a * (lbj - ui) / d
    normal = np.array([-(lbj - ui)[1], (lbj - ui)[0]]) / d
    candidates = [base + h * normal, base - h * normal]
    ubj = min(candidates, key=lambda c: np.linalg.norm(c - ubj0))
    turn = math.atan2(*(ubj - lbj)[::-1]) - math.atan2(*(ubj0 - lbj0)[::-1])
    contact = lbj + rot(np.array([p['wheel'][1], -WHEEL_RADIUS]) - lbj0, turn)
    mount = li + rot(p['shock_lower'][1:] - li, alpha)
    return contact, float(np.linalg.norm(p['shock_upper'][1:] - mount))


def wheel_kinematics(p, step=1e-5):
    """Motion ratio (damper shortening per unit of wheel rise at the contact point) and scrub rate
    (lateral motion of the contact point per unit of wheel rise) at the design pose."""
    (c0, l0), (c1, l1) = front_view(p, -step), front_view(p, step)
    rise = c1[1] - c0[1]
    return abs((l1 - l0) / rise), (c1[0] - c0[0]) / rise


def motion_ratio(p):
    """Damper shortening per unit of wheel rise for small motions."""
    return wheel_kinematics(p)[0]


def axle_sprung_mass(axle):
    """Chassis mass carried by one corner of the axle."""
    length = AXLES['f']['x'] - AXLES['r']['x']
    front = (CHASSIS_COM[0] - AXLES['r']['x']) / length
    return 0.5 * MASS['chassis'] * (front if axle == 'f' else 1.0 - front)


def corner_unsprung_mass():
    """Mass of one corner that moves with the wheel: wheel, upright, tyre carriers and half of the
    arms, tie rod and coilover."""
    return (MASS['wheel'] + MASS['upright'] + 2 * TYRE['carrier_mass'] +
            0.5 * (MASS['lower_arm'] + MASS['upper_arm'] + MASS['tie_rod'] + MASS['shock_body'] + MASS['shock_rod']))


def tyre_load(axle):
    """Static force in the radial tyre spring: everything the tyre carries except the wheel itself."""
    return (axle_sprung_mass(axle) + corner_unsprung_mass() - MASS['wheel']) * G


def spring_damper(axle):
    """Spring rate, damping, preload and static force of the coilovers of one axle."""
    p = corner_points(axle, 1)
    mr = motion_ratio(p)
    m = axle_sprung_mass(axle)
    wheel_rate = m * (2 * math.pi * RIDE_FREQUENCY[axle]) ** 2
    k = wheel_rate / mr ** 2
    damping = DAMPING_RATIO[axle] * 2 * math.sqrt(wheel_rate * m) / mr ** 2
    force = m * G / mr
    preload = force / k
    return dict(motion_ratio=mr, sprung_mass=m, wheel_rate=wheel_rate, stiffness=k, damping=damping,
                static_force=force, preload=preload,
                length=float(np.linalg.norm(p['shock_upper'] - p['shock_lower'])))


def fmt(v):
    # Round away floating-point dust such as 1e-17 so that the URDF reads cleanly.
    return ' '.join(f'{round(float(x), 9) + 0.0:.6g}' for x in np.atleast_1d(v))


class Urdf:
    def __init__(self, name):
        self.name = name
        self.links, self.joints, self.pins, self.equalities = [], [], [], []
        self.tree = []  # (joint, type, parent, child) in declaration order

    def link(self, name, mass, com=(0, 0, 0), inertia=None, body=''):
        inertia = inertia or (0.01 * mass,) * 3
        self.links.append(
            f'  <link name="{name}">\n'
            f'    <inertial>\n      <origin xyz="{fmt(com)}"/>\n      <mass value="{mass:.6g}"/>\n'
            f'      <inertia ixx="{inertia[0]:.6g}" ixy="0" ixz="0" iyy="{inertia[1]:.6g}" iyz="0" '
            f'izz="{inertia[2]:.6g}"/>\n    </inertial>\n{body}  </link>\n')

    def joint(self, name, kind, parent, child, xyz, rpy=(0, 0, 0), axis=None, body=''):
        text = (f'  <joint name="{name}" type="{kind}">\n    <parent link="{parent}"/>\n'
                f'    <child link="{child}"/>\n    <origin xyz="{fmt(xyz)}" rpy="{fmt(rpy)}"/>\n')
        if axis is not None:
            text += f'    <axis xyz="{fmt(axis)}"/>\n'
        self.joints.append(text + body + '  </joint>\n')
        self.tree.append((name, kind, parent, child))

    def coordinate_order(self, root):
        """Joints in RaiSim's generalized-coordinate order: depth first, children in file order."""
        order = []

        def visit(link):
            for joint in self.tree:
                if joint[2] == link:
                    order.append(joint)
                    visit(joint[3])
        visit(root)
        return order

    def text(self, root, base_height):
        # Every joint is at zero (spherical joints at the identity) in the design pose, in which
        # all loops are closed.
        nominal = [0, 0, base_height, 1, 0, 0, 0]
        for _, kind, _, _ in self.coordinate_order(root):
            nominal += [1, 0, 0, 0] if kind == 'spherical' else [0]
        out = [f'<?xml version="1.0"?>\n<!-- Off-road buggy of rayrai_offroad_buggy. Generated by '
               f'examples/tools/prepare_offroad_buggy_assets.py from examples/tools/offroad_buggy_design.py; '
               f'edit those and regenerate. -->\n<robot name="{self.name}">\n'] + self.links + self.joints
        out.append(f'  <constraints nominal_config="{fmt(nominal)}">\n')
        for body1, body2, anchor in self.pins:
            out.append(f'    <pin body1="{body1}" body2="{body2}" anchor="{fmt(anchor)}"/>\n')
        for body1, body2, anchor, axes in self.equalities:
            out.append(f'    <equality body1="{body1}" body2="{body2}" anchor="{fmt(anchor)}">\n')
            out += [f'      <axis xyz="{fmt(axis)}"/>\n' for axis in axes]
            out.append('    </equality>\n')
        out.append('  </constraints>\n</robot>\n')
        return ''.join(out)


def visual(mesh, xyz=(0, 0, 0), rpy=(0, 0, 0), scale=(1, 1, 1), material=None, colour=None):
    text = (f'    <visual>\n      <origin xyz="{fmt(xyz)}" rpy="{fmt(rpy)}"/>\n'
            f'      <geometry><mesh filename="{mesh}" scale="{fmt(scale)}"/></geometry>\n')
    if material:
        text += f'      <material name="{material}"><color rgba="{fmt(colour)}"/></material>\n'
    return text + '    </visual>\n'


def drive_shaft_visuals(meshes):
    text = ''
    for name, side in [('rl', 1), ('rr', -1)]:
        inner = np.array([-1.3571, 0.30 * side, 0.0])
        outer = corner_points('r', side)['wheel'] - np.array([0.0, 0.115 * side, 0.0])
        axis = outer - inner
        length = np.linalg.norm(axis)
        direction = axis / length
        # Rz(yaw) Ry(pitch) maps the visual's +Z axis onto the shaft.
        rpy = (0.0, math.acos(direction[2]), math.atan2(direction[1], direction[0]))
        text += visual(f'{meshes}/drive_shaft.gltf', (inner + outer) / 2, rpy,
                       (1, 1, length - 0.1), f'{name}_shaft', (0.55, 0.56, 0.58, 1))
        for end, centre in [('inner', inner + direction * 0.045), ('outer', outer - direction * 0.045)]:
            text += (f'    <visual>\n      <origin xyz="{fmt(centre)}" rpy="{fmt(rpy)}"/>\n'
                     '      <geometry><capsule radius="0.042" length="0.06"/></geometry>\n'
                     f'      <material name="{name}_{end}_boot"><color rgba="0.02 0.02 0.02 1"/></material>\n'
                     '    </visual>\n')
    return text


def collision(geometry, material, xyz=(0, 0, 0), rpy=(0, 0, 0)):
    return (f'    <collision>\n      <origin xyz="{fmt(xyz)}" rpy="{fmt(rpy)}"/>\n'
            f'      <geometry>{geometry}</geometry>\n      <material name="{material}"/>\n    </collision>\n')


# Chassis collision boxes (centre, size): floor pan, nose, cockpit and roll cage, rear deck.
CHASSIS_BOXES = [((-0.12, 0.0, 0.06), (2.30, 1.50, 0.39)), ((1.45, 0.0, 0.07), (0.95, 1.10, 0.34)),
                 ((-0.30, 0.0, 0.62), (1.70, 1.60, 0.74)), ((-1.35, 0.0, 0.55), (0.60, 1.30, 0.90))]


def rod_inertia(mass, length):
    i = mass * length ** 2 / 12
    return (i, i, i)


def build_urdf(steering_wheel, meshes='meshes'):
    """URDF text of the buggy. steering_wheel = (centre, axis) in the chassis frame, the axis
    pointing at the driver, so that a positive angle turns the wheel counter-clockwise."""
    u = Urdf('offroad_buggy')
    body = visual(f'{meshes}/chassis.gltf') + drive_shaft_visuals(meshes) + visual(f'{meshes}/steering_rack_housing.gltf', rack_origin())
    body += ''.join(collision(f'<box size="{fmt(size)}"/>', 'body', centre) for centre, size in CHASSIS_BOXES)
    u.link('chassis', MASS['chassis'], CHASSIS_COM, CHASSIS_INERTIA, body)
    centre, axis = steering_wheel
    u.joint('steering_wheel', 'revolute', 'chassis', 'steering_wheel', centre, axis=axis,
            body='    <limit effort="300" velocity="40" lower="-7.5" upper="7.5"/>\n'
                 '    <dynamics damping="0.3"/>\n')
    u.link('steering_wheel', MASS['steering_wheel'], inertia=(0.03, 0.03, 0.03),
           body=visual(f'{meshes}/steering_wheel.gltf'))
    # A positive steering-wheel angle moves the rack to the right, which turns the car left.
    u.joint('steering_rack', 'prismatic', 'chassis', 'steering_rack', rack_origin(), axis=(0, 1, 0),
            body=f'    <limit effort="30000" velocity="2" lower="{-RACK_TRAVEL}" upper="{RACK_TRAVEL}"/>\n'
                 f'    <mimic joint="steering_wheel" multiplier="{-PINION_RADIUS}" offset="0"/>\n')
    u.link('steering_rack', MASS['rack'], inertia=(0.06, 0.002, 0.06), body=visual(f'{meshes}/steering_rack.gltf'))
    for c, axle, side in CORNERS:
        p = corner_points(axle, side)
        sd = spring_damper(axle)
        hinge = '    <limit effort="0" velocity="100" lower="-3.1" upper="3.1"/>\n    <dynamics damping="2"/>\n'
        # Lower arm and upright.
        arm = p['lbj'] - p['lower_inner']
        u.joint(f'{c}_lower_arm', 'revolute', 'chassis', f'{c}_lower_arm', p['lower_inner'], axis=(1, 0, 0),
                body=hinge)
        u.link(f'{c}_lower_arm', MASS['lower_arm'], arm * 0.55, rod_inertia(MASS['lower_arm'], 0.5),
               visual(f'{meshes}/{c}_lower_arm.gltf'))
        u.joint(f'{c}_upright', 'spherical', f'{c}_lower_arm', f'{c}_upright', arm,
                body='    <dynamics damping="0.05"/>\n')
        upright = visual(f'{meshes}/{c}_upright.gltf')
        if AXLES[axle]['steer']:
            upright += visual(f'{meshes}/{c}_fender.gltf')  # the front fenders ride on the uprights
        u.link(f'{c}_upright', MASS['upright'], p['wheel'] - p['lbj'] - [0, side * 0.05, 0], (0.05, 0.05, 0.05),
               upright)
        # Tyre compliance: the wheel rides on a radial and a lateral spring-damper. A rigid tyre
        # cannot scrub sideways as the wishbones swing, so friction would lock the suspension.
        radial_preload = tyre_load(axle) / TYRE['radial_stiffness']
        u.joint(f'{c}_tyre_radial', 'prismatic', f'{c}_upright', f'{c}_tyre_radial', p['wheel'] - p['lbj'],
                axis=(0, 0, 1),
                body=f'    <limit effort="0" velocity="10" lower="{TYRE["radial_travel"][0]}" upper="{TYRE["radial_travel"][1]}"/>\n'
                     f'    <dynamics stiffness="{TYRE["radial_stiffness"]:.6g}" spring_mount="{-radial_preload:.6g}" '
                     f'damping="{TYRE["radial_damping"]:.6g}"/>\n')
        u.link(f'{c}_tyre_radial', TYRE['carrier_mass'], inertia=(1e-3, 1e-3, 1e-3))
        u.joint(f'{c}_tyre_lateral', 'prismatic', f'{c}_tyre_radial', f'{c}_tyre_lateral', (0, 0, 0), axis=(0, 1, 0),
                body=f'    <limit effort="0" velocity="10" lower="{-TYRE["lateral_travel"]}" upper="{TYRE["lateral_travel"]}"/>\n'
                     f'    <dynamics stiffness="{TYRE["lateral_stiffness"]:.6g}" spring_mount="0" '
                     f'damping="{TYRE["lateral_damping"]:.6g}"/>\n')
        u.link(f'{c}_tyre_lateral', TYRE['carrier_mass'], inertia=(1e-3, 1e-3, 1e-3))
        # Wheel: a cylinder collider spinning about the upright's y axis, narrower than the tread
        # like the contact patch of a crowned tyre, so that camber does not rock it far onto an edge.
        tyre = collision(f'<cylinder radius="{WHEEL_RADIUS}" length="{TYRE["contact_width"] * AXLES[axle]["width"]:.4g}"/>', 'tire',
                         rpy=(math.pi / 2, 0, 0))
        u.joint(f'{c}_wheel', 'continuous', f'{c}_tyre_lateral', f'{c}_wheel', (0, 0, 0), axis=(0, 1, 0),
                body='    <dynamics damping="0.3"/>\n')
        u.link(f'{c}_wheel', MASS['wheel'], inertia=WHEEL_INERTIA,
               body=visual(f'{meshes}/{c}_wheel.gltf') + visual(f'{meshes}/{c}_disc.gltf') + tyre)
        # Upper arm, pinned to the upright at the upper ball joint.
        upper = p['ubj'] - p['upper_inner']
        u.joint(f'{c}_upper_arm', 'revolute', 'chassis', f'{c}_upper_arm', p['upper_inner'], axis=(1, 0, 0),
                body=hinge)
        u.link(f'{c}_upper_arm', MASS['upper_arm'], upper * 0.55, rod_inertia(MASS['upper_arm'], 0.4),
               visual(f'{meshes}/{c}_upper_arm.gltf'))
        u.pins.append((f'{c}_upper_arm', f'{c}_upright', upper))
        # Tie rod (rack in front, toe link at the rear), pinned to the steering arm.
        parent, origin = ('steering_rack', p['itr'] - rack_origin()) if AXLES[axle]['steer'] else ('chassis', p['itr'])
        rod = p['otr'] - p['itr']
        u.joint(f'{c}_tie_rod', 'spherical', parent, f'{c}_tie_rod', origin, body='    <dynamics damping="0.05"/>\n')
        u.link(f'{c}_tie_rod', MASS['tie_rod'], rod * 0.5, rod_inertia(MASS['tie_rod'], float(np.linalg.norm(rod))),
               visual(f'{meshes}/{c}_tie_rod.gltf'))
        u.pins.append((f'{c}_tie_rod', f'{c}_upright', rod))
        # Coilover. Joint coordinate: extension of the damper from its design length.
        u.joint(f'{c}_shock_body', 'revolute', 'chassis', f'{c}_shock_body', p['shock_upper'],
                rpy=(shock_angle(p), 0, 0), axis=(1, 0, 0),
                body='    <limit effort="0" velocity="100" lower="-3.1" upper="3.1"/>\n    <dynamics damping="0.5"/>\n')
        u.link(f'{c}_shock_body', MASS['shock_body'], (0, 0, -0.09), (0.02, 0.02, 0.004),
               visual(f'{meshes}/coilover_body.gltf') + visual(f'{meshes}/coil_spring.gltf', material=f'{c}_coil', colour=(0.10, 0.80, 0.30, 1)))
        low, high = SHOCK_TRAVEL
        u.joint(f'{c}_shock', 'prismatic', f'{c}_shock_body', f'{c}_shock_rod', (0, 0, 0), axis=(0, 0, -1),
                body=f'    <limit effort="0" velocity="10" lower="{low}" upper="{high}"/>\n'
                     f'    <dynamics stiffness="{sd["stiffness"]:.6g}" spring_mount="{sd["preload"]:.6g}" '
                     f'damping="{sd["damping"]:.6g}"/>\n')
        u.link(f'{c}_shock_rod', MASS['shock_rod'], (0, 0, -sd['length'] + 0.06), (0.006, 0.006, 0.001),
               visual(f'{meshes}/coilover_rod.gltf'))
        u.equalities.append((f'{c}_lower_arm', f'{c}_shock_rod', p['shock_lower'] - p['lower_inner'],
                             [(0, 1, 0), (0, 0, 1)]))
    return u.text('chassis', WHEEL_RADIUS)


def summary():
    lines = []
    for axle in ('f', 'r'):
        sd = spring_damper(axle)
        lines.append(f"{axle}: motion ratio {sd['motion_ratio']:.3f}, spring {sd['stiffness'] / 1000:.1f} kN/m, "
                     f"damper {sd['damping']:.0f} N s/m, static force {sd['static_force']:.0f} N, "
                     f"preload {sd['preload'] * 1000:.1f} mm, design length {sd['length'] * 1000:.0f} mm")
    return '\n'.join(lines)
