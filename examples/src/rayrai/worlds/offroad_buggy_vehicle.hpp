#pragma once
// The off-road buggy of rayrai_offroad_buggy: driver inputs, the electric rear
// drive with a limited-slip differential, the brakes, aerodynamic drag and
// rolling resistance, and the telemetry of the four corners.
//
// The mechanism itself (double wishbones, coilovers, tie rods, rack and
// pinion, tyre compliance) is entirely in rsc/offroad_buggy/buggy.urdf: a
// spanning tree closed by pin, equality and mimic constraints. Nothing here
// moves a suspension part.
#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "raisim/World.hpp"

namespace offroad {

struct DriverInput {
  double throttle = 0;   ///< 0..1, forward (or backward with reverse)
  double brake = 0;      ///< 0..1
  double steer = 0;      ///< -1 (right) .. 1 (left)
  bool reverse = false;  ///< drive backwards
  bool handbrake = false;
  bool parkingHold = true;  ///< hold the wheels when the car stands still without throttle
};

/// Telemetry of one corner, averaged over the steps of a frame.
struct CornerTelemetry {
  double shockExtension = 0;                            ///< damper extension from the design length [m] (+ = droop)
  double compression = 0;                               ///< 0 at full droop .. 1 on the bump stop
  double springLength = 0;                              ///< coil spring length between its seats [m]
  double springForce = 0;                               ///< [N], positive pushes the seats apart
  double damperVelocity = 0;                            ///< [m/s], positive extends
  double damperForce = 0;                               ///< [N], positive pushes the eyes apart
  double wheelLoad = 0;                                 ///< normal force of the ground on the tyre [N]
  double wheelSpeed = 0;                                ///< [rad/s]
  double slip = 0;                                      ///< longitudinal slip ratio
  double steerAngle = 0;                                ///< wheel yaw relative to the chassis [rad]
  double camber = 0;                                    ///< wheel camber relative to the chassis [rad], negative = top inwards
  double driveTorque = 0;                               ///< [Nm]
  double tyreDeflection = 0;                            ///< radial compression of the tyre [m]
  double tyreLateral = 0;                               ///< lateral deflection of the tyre [m]
  Eigen::Vector2d contact2d = Eigen::Vector2d::Zero();  ///< load centre of the contact patch in the heading frame (x, y) [m]
  bool contact = false;
};

struct Telemetry {
  std::array<CornerTelemetry, 4> corners;
  double speed = 0;  ///< forward speed [m/s]
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();  ///< chassis-frame acceleration of the centre of mass
  double rollAngle = 0, pitchAngle = 0;                    ///< chassis attitude [rad]
  double motorPower = 0;                                   ///< [W]
  double maxLoopError = 0;                                 ///< largest anchor separation of the loop constraints [m]
  double steeringWheelAngle = 0;                           ///< [rad]
  double rackPosition = 0;                                 ///< [m]
  double comHeight = 0;                                    ///< centre of mass above the mean contact point [m]
  Eigen::Vector2d com2d = Eigen::Vector2d::Zero();         ///< centre of mass in the heading frame (x, y) [m]
};

class Buggy {
 public:
  static constexpr const char* kCornerNames[4] = {"fl", "fr", "rl", "rr"};
  // Electric drive unit on the rear axle, torques at the wheels.
  static constexpr double kMaxAxleTorque = 1500.0;  // [Nm]
  static constexpr double kMaxPower = 75000.0;      // [W]
  static constexpr double kMaxWheelSpeed = 78.0;    // [rad/s], about 100 km/h
  static constexpr double kReverseTorque = 600.0;   // [Nm]
  static constexpr double kLsdCoupling = 60.0;      // viscous limited-slip coupling [Nm s/rad]
  // Brakes: implicit viscous brake torque per wheel per rad/s, 60/40 front/rear,
  // and a held wheel (stiff PD on the wheel angle) near standstill.
  static constexpr double kBrakeDamping[2] = {420.0, 280.0};
  static constexpr double kHoldSpeed = 0.6;  // [m/s]
  static constexpr double kHoldStiffness = 6000.0, kHoldDamping = 400.0;
  static constexpr double kDragArea = 0.95;  // Cd A [m^2]
  static constexpr double kRollingResistance = 0.018;
  static constexpr double kSteeringWheelLock = 5.8;  // [rad], 332 degrees: the rack travel over the pinion radius
  static constexpr double kSteeringRate = 7.0;       // [rad/s] of the steering wheel

  Buggy(raisim::World& world, const std::string& urdf) : world_(world) {
    system_ = world.addArticulatedSystem(urdf);
    system_->setName("offroad_buggy");
    dof_ = system_->getDOF();
    for (size_t i = 0; i < system_->getMass().size(); ++i) mass_ += system_->getMass()[i];
    baseDamping_ = system_->getJointDamping().e();
    for (int i = 0; i < 4; ++i) {
      const std::string c = kCornerNames[i];
      Corner& k = corners_[i];
      k.wheelBody = system_->getBodyIdx(c + "_wheel");
      k.uprightBody = system_->getBodyIdx(c + "_upright");
      k.shockBodyBody = system_->getBodyIdx(c + "_shock_body");
      k.wheelGc = system_->getJoint(c + "_wheel").getIdxInGeneralizedCoordinate();
      k.wheelGv = system_->getGeneralizedVelocityIndex(c + "_wheel");
      k.shockGc = system_->getJoint(c + "_shock").getIdxInGeneralizedCoordinate();
      k.shockGv = system_->getGeneralizedVelocityIndex(c + "_shock");
      k.damping = baseDamping_[k.shockGv];
      k.tyreRadialGc = system_->getJoint(c + "_tyre_radial").getIdxInGeneralizedCoordinate();
      k.tyreLateralGc = system_->getJoint(c + "_tyre_lateral").getIdxInGeneralizedCoordinate();
      k.tyreDamping = baseDamping_[system_->getGeneralizedVelocityIndex(c + "_tyre_radial")];
      const size_t tyreBody = system_->getBodyIdx(c + "_tyre_radial");
      const size_t shockBody = system_->getBodyIdx(c + "_shock_rod");
      for (const auto& spring : system_->getSprings()) {
        if (spring.childBodyId == shockBody) {
          k.stiffness = spring.stiffness;
          k.springMount = spring.q_ref[0];
        }
        if (spring.childBodyId == tyreBody) {
          k.tyreStiffness = spring.stiffness;
          k.tyreMount = spring.q_ref[0];
        }
      }
      if (k.stiffness <= 0) throw std::runtime_error("No coil spring on " + c + "_shock in " + urdf);
      k.limits = {system_->getJointLimits()[k.shockGv][0], system_->getJointLimits()[k.shockGv][1]};
      // The tyre is the wheel's only collision body: never collide it with the chassis.
      system_->ignoreCollisionBetween(0, k.wheelBody);
    }
    system_->updateSelfCollisionCache(world);
    for (int i = 0; i < 4; ++i) wheelRadius_ = std::max(wheelRadius_, system_->getCollisionBody(std::string(kCornerNames[i]) + "_wheel/0").shapeParam[0]);
    steeringGc_ = system_->getJoint("steering_wheel").getIdxInGeneralizedCoordinate();
    steeringGv_ = system_->getGeneralizedVelocityIndex("steering_wheel");
    rackGc_ = system_->getJoint("steering_rack").getIdxInGeneralizedCoordinate();
    designPose_ = system_->getGeneralizedCoordinate().e();
    system_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    pTarget_ = designPose_;
    vTarget_.setZero(dof_);
    pGain_.setZero(dof_);
    dGain_.setZero(dof_);
    // Power steering: the steering wheel follows the driver's angle; the rack follows the wheel.
    pGain_[steeringGv_] = 500.0;
    dGain_[steeringGv_] = 25.0;
    system_->setPdGains(pGain_, dGain_);
    system_->setPdTarget(pTarget_, vTarget_);
    tau_.setZero(dof_);
  }

  raisim::ArticulatedSystem* system() const { return system_; }
  double mass() const { return mass_; }
  double wheelRadius() const { return wheelRadius_; }
  double springMount(int corner) const { return corners_[corner].springMount; }
  double stiffness(int corner) const { return corners_[corner].stiffness; }
  double damperCoefficient(int corner) const { return corners_[corner].damping; }
  double tyreStiffness(int corner) const { return corners_[corner].tyreStiffness; }
  double tyreDamping(int corner) const { return corners_[corner].tyreDamping; }
  std::array<double, 2> shockLimits(int corner) const { return corners_[corner].limits; }
  size_t shockBodyIndex(int corner) const { return corners_[corner].shockBodyBody; }
  size_t wheelBodyIndex(int corner) const { return corners_[corner].wheelBody; }
  size_t uprightBodyIndex(int corner) const { return corners_[corner].uprightBody; }
  const Telemetry& telemetry() const { return telemetry_; }

  /// Put the car on the ground at (x, y) facing heading, in its design pose, at rest.
  template <class GroundHeight>
  void place(double x, double y, double heading, GroundHeight&& ground) {
    Eigen::VectorXd q = designPose_;
    const double c = std::cos(heading), s = std::sin(heading);
    // Lift the car so that the highest ground under its wheels is just below the tyres.
    double z = -1e9;
    for (double dx : {-1.4, 1.4})
      for (double dy : {-1.1, 1.1}) z = std::max(z, ground(x + c * dx - s * dy, y + s * dx + c * dy));
    q[0] = x;
    q[1] = y;
    q[2] = z + wheelRadius_ + 0.015;
    q[3] = std::cos(heading / 2);
    q[4] = 0;
    q[5] = 0;
    q[6] = std::sin(heading / 2);
    system_->setState(q, Eigen::VectorXd::Zero(dof_));
    pTarget_ = q;
    for (auto& k : corners_) k.holdAngle = q[k.wheelGc];
    steeringAngle_ = 0;
    telemetry_ = {};
  }

  /// Set the joint torques, the brakes and the drag for the next step; call before World::integrate().
  void applyControls(const DriverInput& input, double dt) {
    const auto& gv = system_->getGeneralizedVelocity();
    const auto& gc = system_->getGeneralizedCoordinate();
    raisim::Mat<3, 3> rot;
    system_->getBaseOrientation(rot);
    const Eigen::Vector3d forward = rot.e().col(0);
    const Eigen::Vector3d velocity(gv[0], gv[1], gv[2]);
    const double speed = forward.dot(velocity);

    // Steering: speed-sensitive lock, rate-limited like a power-steering motor.
    const double lock = kSteeringWheelLock / (1.0 + speed * speed / 500.0);
    const double target = std::clamp(input.steer, -1.0, 1.0) * lock;
    steeringAngle_ += std::clamp(target - steeringAngle_, -kSteeringRate * dt, kSteeringRate * dt);
    pTarget_[steeringGc_] = steeringAngle_;

    // Drive: torque limited by the motor's peak torque and power and fading at its top speed.
    tau_.setZero();
    const double wl = gv[corners_[2].wheelGv], wr = gv[corners_[3].wheelGv];
    const double wheelSpeed = 0.5 * (wl + wr);
    double axle = 0;
    if (input.throttle > 0) {
      const double direction = input.reverse ? -1.0 : 1.0;
      const double omega = std::abs(wheelSpeed);
      const double peak = input.reverse ? kReverseTorque : kMaxAxleTorque;
      double available = std::min(peak, kMaxPower / std::max(omega, 1.0));
      if (direction * wheelSpeed > 0) available *= std::clamp((kMaxWheelSpeed - omega) / 8.0, 0.0, 1.0);
      axle = direction * input.throttle * available;
    }
    // Open differential (equal torques) with a viscous limited-slip coupling.
    const double bias = std::clamp(kLsdCoupling * (wl - wr), -0.5 * std::abs(axle) - 120.0, 0.5 * std::abs(axle) + 120.0);
    driveTorque_ = {0.0, 0.0, 0.5 * axle - bias, 0.5 * axle + bias};
    motorPower_ = axle * wheelSpeed;

    // Brakes and parking hold.
    Eigen::VectorXd damping = baseDamping_;
    const bool slow = std::abs(speed) < kHoldSpeed;
    const bool hold = slow && (input.brake > 0 || input.handbrake || (input.throttle == 0 && input.parkingHold));
    for (int i = 0; i < 4; ++i) {
      Corner& k = corners_[i];
      const bool rear = i >= 2;
      double brake = input.brake;
      if (rear && input.handbrake) brake = 1.0;
      const bool held = hold && (brake > 0 || (input.throttle == 0 && input.parkingHold));
      if (held) {
        if (!k.held) k.holdAngle = gc[k.wheelGc];
        pGain_[k.wheelGv] = kHoldStiffness;
        dGain_[k.wheelGv] = kHoldDamping;
        pTarget_[k.wheelGc] = k.holdAngle;
      } else {
        pGain_[k.wheelGv] = 0;
        dGain_[k.wheelGv] = 0;
        damping[k.wheelGv] += brake * kBrakeDamping[rear ? 1 : 0] * (rear && input.handbrake ? 3.0 : 1.0);
      }
      k.held = held;
      // Rolling resistance: a torque against the rolling direction, proportional to the load.
      const double omega = gv[k.wheelGv];
      tau_[k.wheelGv] = driveTorque_[i] - kRollingResistance * k.load * wheelRadius_ * std::tanh(omega / 0.5);
    }
    system_->setJointDamping(damping);
    system_->setPdGains(pGain_, dGain_);
    system_->setPdTarget(pTarget_, vTarget_);
    system_->setGeneralizedForce(tau_);
    // Aerodynamic drag at the chassis centre of mass.
    const Eigen::Vector3d drag = -0.5 * 1.2 * kDragArea * velocity.norm() * velocity;
    system_->setExternalForce(0, raisim::Vec<3>{-0.14, 0.0, 0.10}, raisim::Vec<3>{drag.x(), drag.y(), drag.z()});
  }

  /// Accumulate contact forces and constraint errors of the step just taken; call after World::integrate().
  void accumulate(double dt) {
    const size_t self = system_->getIndexInWorld();
    // Contact positions in the heading frame: the chassis origin with the car's yaw only, so that
    // roll and pitch do not skew the horizontal lever arms.
    const Eigen::Matrix3d heading = headingFrame();
    const auto& gc = system_->getGeneralizedCoordinate();
    const Eigen::Vector3d base(gc[0], gc[1], gc[2]);
    for (auto& contact : system_->getContacts()) {
      if (contact.skip() || contact.getPairObjectIndex() == self) continue;
      for (auto& k : corners_)
        if (contact.getlocalBodyIndex() == k.wheelBody && contact.getImpulsePtr()) {
          Eigen::Vector3d f = contact.getContactFrame().e().transpose() * contact.getImpulse().e() / dt;
          if (!contact.isObjectA()) f = -f;
          const double normal = f.dot(contact.getNormal().e());
          k.loadSum += normal;
          k.contactZ += contact.getPosition()[2];
          k.contactSum += normal * (heading.transpose() * (contact.getPosition().e() - base)).head<2>();
          ++k.contactCount;
        }
    }
    for (const auto& problem : *world_.getContactProblem())
      if (problem.rank == raisim::contact::rank::PROJECTED_PIN_CONSTRAINT) loopError_ = std::max(loopError_, problem.pinError_W.e().norm());
    ++steps_;
  }

  /// Average the accumulated step data into the telemetry; call once per frame.
  void updateTelemetry(double frameTime) {
    const auto& gc = system_->getGeneralizedCoordinate();
    const auto& gv = system_->getGeneralizedVelocity();
    raisim::Mat<3, 3> rot;
    system_->getBaseOrientation(rot);
    const Eigen::Matrix3d R = rot.e();
    Telemetry& t = telemetry_;
    t.velocity = Eigen::Vector3d(gv[0], gv[1], gv[2]);
    t.speed = R.col(0).dot(t.velocity);
    // Centre-of-mass acceleration from its velocity, low-pass filtered.
    const Eigen::Vector3d v = system_->getLinearMomentum().e() / mass_;
    if (frameTime > 0 && hasLastComVelocity_) {
      const Eigen::Vector3d a = R.transpose() * (v - lastComVelocity_) / frameTime;
      t.acceleration += (a - t.acceleration) * std::min(1.0, frameTime / 0.25);
    }
    lastComVelocity_ = v;
    hasLastComVelocity_ = true;
    t.rollAngle = std::atan2(R(2, 1), R(2, 2));
    t.pitchAngle = -std::asin(std::clamp(R(2, 0), -1.0, 1.0));
    t.motorPower = motorPower_;
    t.maxLoopError = loopError_;
    t.steeringWheelAngle = gc[steeringGc_];
    t.rackPosition = gc[rackGc_];
    double contactZ = 0;
    int contacts = 0;
    for (int i = 0; i < 4; ++i) {
      Corner& k = corners_[i];
      CornerTelemetry& c = t.corners[i];
      const double q = gc[k.shockGc], qd = gv[k.shockGv];
      c.shockExtension = q;
      c.compression = std::clamp((k.limits[1] - q) / (k.limits[1] - k.limits[0]), 0.0, 1.0);
      c.springForce = k.stiffness * (k.springMount - q);
      c.damperVelocity = qd;
      c.damperForce = -k.damping * qd;
      const double load = steps_ ? k.loadSum / steps_ : 0.0;
      c.wheelLoad += (load - c.wheelLoad) * std::min(1.0, frameTime / 0.12);
      c.contact = k.contactCount > 0;
      if (k.contactCount) {
        contactZ += k.contactZ / k.contactCount;
        ++contacts;
      }
      k.load = c.wheelLoad;
      if (k.loadSum > 1e-6) c.contact2d += (k.contactSum / k.loadSum - c.contact2d) * std::min(1.0, frameTime / 0.12);
      c.tyreDeflection = gc[k.tyreRadialGc] - k.tyreMount;
      c.tyreLateral = gc[k.tyreLateralGc];
      c.wheelSpeed = gv[k.wheelGv];
      raisim::Mat<3, 3> upright;
      raisim::Vec<3> position;
      system_->getBodyPose(k.uprightBody, upright, position);
      const Eigen::Matrix3d local = R.transpose() * upright.e();
      c.steerAngle = std::atan2(local(1, 0), local(0, 0));
      const double side = i % 2 == 0 ? 1.0 : -1.0;
      c.camber = side * std::asin(std::clamp(local(2, 1), -1.0, 1.0));
      const double groundSpeed = t.speed;
      const double rolling = c.wheelSpeed * wheelRadius_;
      c.slip = std::abs(groundSpeed) > 1.0 ? (rolling - groundSpeed) / std::abs(groundSpeed) : 0.0;
      c.driveTorque = driveTorque_[i];
      k.loadSum = 0;
      k.contactZ = 0;
      k.contactCount = 0;
      k.contactSum.setZero();
    }
    raisim::Vec<3> com = system_->getCOM();
    if (contacts) t.comHeight = com[2] - contactZ / contacts;
    t.com2d = (headingFrame().transpose() * (com.e() - Eigen::Vector3d(gc[0], gc[1], gc[2]))).head<2>();
    loopError_ = 0;
    steps_ = 0;
  }

 private:
  /// Rotation by the car's heading about the vertical.
  Eigen::Matrix3d headingFrame() const {
    raisim::Mat<3, 3> rot;
    system_->getBaseOrientation(rot);
    const double yaw = std::atan2(rot(1, 0), rot(0, 0));
    return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  }

  struct Corner {
    size_t wheelBody = 0, uprightBody = 0, shockBodyBody = 0;
    size_t wheelGc = 0, wheelGv = 0, shockGc = 0, shockGv = 0;
    double stiffness = 0, springMount = 0, damping = 0;
    size_t tyreRadialGc = 0, tyreLateralGc = 0;
    double tyreStiffness = 0, tyreMount = 0, tyreDamping = 0;
    Eigen::Vector2d contactSum = Eigen::Vector2d::Zero();
    std::array<double, 2> limits{};
    double holdAngle = 0;
    bool held = false;
    double load = 0, loadSum = 0, contactZ = 0;
    int contactCount = 0;
  };

  raisim::World& world_;
  raisim::ArticulatedSystem* system_ = nullptr;
  size_t dof_ = 0;
  double mass_ = 0, wheelRadius_ = 0;
  std::array<Corner, 4> corners_{};
  size_t steeringGc_ = 0, steeringGv_ = 0, rackGc_ = 0;
  Eigen::VectorXd designPose_, baseDamping_, pTarget_, vTarget_, pGain_, dGain_, tau_;
  double steeringAngle_ = 0;
  std::array<double, 4> driveTorque_{};
  double motorPower_ = 0;
  double loopError_ = 0;
  int steps_ = 0;
  Eigen::Vector3d lastComVelocity_ = Eigen::Vector3d::Zero();
  bool hasLastComVelocity_ = false;
  Telemetry telemetry_;
};

}  // namespace offroad
