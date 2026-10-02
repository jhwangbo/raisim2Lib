// Actuators and the motor operating region (EM-MOR) of their geared DC motors, defined in the URDF.
//
// A fixed-base quadruped rig is actuated randomly. rsc/motorOperatingRegion/quadruped_rig.urdf
// attaches two actuators to every leg with <actuator> elements that link actuator files: a geared
// hip abduction motor, and the hip and knee motors coupled like KAIST Hound. For each of the twelve
// motors, the plots show the operating region in the motor's torque-speed plane (green), the
// Box-MOR rectangle of peak torque and no-load speed (dashed), and the operating points of the last
// second. Points outside the region by more than 1% of the peak torque are red and counted, so you
// can check that RaiSim keeps every motor inside its region. Untick "Enforce EM-MOR" to see what
// the URDF effort limits (Box-MOR) alone would do.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "rayrai/example_common.hpp"
#include "rayrai_example_compat.hpp"
#include "rayrai_example_resources.hpp"
#include "raisim/World.hpp"

namespace {

constexpr double kTolerance = 0.01;    // operating points outside by more than this x peak torque
constexpr double kTrailSeconds = 1.;

// Random commands: position targets, velocity sweeps beyond the no-load speed, or raw torques.
enum class Mode : int { MIXED = 0, POSITION = 1, VELOCITY = 2, TORQUE = 3 };

class RandomActuator {
 public:
  explicit RandomActuator(raisim::ArticulatedSystem* robot)
      : robot_(robot), effort_(robot->getActuationUpperLimits().e()) {
    robot_->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
  }

  Mode mode = Mode::MIXED;
  float period = 0.4f;
  float scale = 1.f;

  void update(double time) {
    if (time < nextChange_) return;
    std::uniform_real_distribution<double> unit(-1., 1.), jitter(0.5, 1.5);
    nextChange_ = time + period * jitter(rng_);
    const Mode active = mode == Mode::MIXED ? static_cast<Mode>(1 + int(rng_() % 3)) : mode;

    const int dof = int(robot_->getDOF());
    Eigen::VectorXd kp = Eigen::VectorXd::Zero(dof), kd = Eigen::VectorXd::Zero(dof);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(dof), u = Eigen::VectorXd::Zero(dof);
    Eigen::VectorXd tau = Eigen::VectorXd::Zero(dof);
    for (int i = 0; i < dof; ++i) {
      const bool abduction = i % 3 == 0;  // has position limits, so it gets no velocity sweeps
      const Mode jointMode = abduction && active == Mode::VELOCITY ? Mode::POSITION : active;
      if (jointMode == Mode::POSITION) {
        kp[i] = abduction ? 40. : 60.;
        kd[i] = 1.;
        q[i] = scale * unit(rng_) * (abduction ? 0.5 : 2.5);
      } else if (jointMode == Mode::VELOCITY) {
        // up to 1.5x the no-load speed: the voltage limit and, on reversal, regenerative braking
        kd[i] = 3.;
        u[i] = scale * unit(rng_) * 36.;
      } else {
        tau[i] = scale * unit(rng_) * 1.5 * effort_[i];
      }
    }
    robot_->setPdGains(kp, kd);
    robot_->setPdTarget(q, u);
    robot_->setGeneralizedForce(tau);
  }

  void restart() { nextChange_ = 0.; }

 private:
  raisim::ArticulatedSystem* robot_;
  Eigen::VectorXd effort_;
  std::mt19937 rng_{42};
  double nextChange_ = 0.;
};

// Operating points of one motor: the step-averaged motor speed and the applied motor torque.
struct Sample {
  float speed = 0.f, torque = 0.f;
  raisim::MotorSaturation saturation = raisim::MotorSaturation::NONE;
  bool outside = false;
};

struct MotorTrace {
  std::vector<Sample> ring = std::vector<Sample>(5000);
  size_t head = 0, count = 0, samples = 0, saturated = 0, outside = 0;
  double maxOutside = 0.;  // [Nm]

  void add(const raisim::MotorState& state, const raisim::DcMotorParameters& motor) {
    // The torque is constant over a step while the speed changes, so compare it with the region at
    // the step-averaged speed.
    const double violation =
        raisim::motorOperatingRegionViolation(motor, state.averageSpeed, state.torque);
    Sample& sample = ring[head];
    sample.speed = float(state.averageSpeed);
    sample.torque = float(state.torque);
    sample.saturation = state.saturation;
    sample.outside = violation > kTolerance * motor.peakTorque;
    head = (head + 1) % ring.size();
    count = std::min(count + 1, ring.size());
    ++samples;
    saturated += state.saturation != raisim::MotorSaturation::NONE;
    outside += sample.outside;
    maxOutside = std::max(maxOutside, violation);
  }

  void reset() { *this = MotorTrace(); }
};

ImU32 sampleColor(const Sample& sample) {
  if (sample.outside) return IM_COL32(255, 70, 70, 255);
  if (sample.saturation == raisim::MotorSaturation::PEAK_TORQUE) return IM_COL32(255, 175, 60, 230);
  if (sample.saturation == raisim::MotorSaturation::VOLTAGE) return IM_COL32(205, 125, 255, 230);
  return IM_COL32(110, 175, 255, 200);
}

// fileVoltage fixes the speed axis, so that a lower bus voltage visibly shrinks the region.
void drawMotorPlot(const std::string& name, const raisim::DcMotorParameters& motor,
                   double fileVoltage, const MotorTrace& trace, size_t trail, ImVec2 size) {
  ImDrawList* draw = ImGui::GetWindowDrawList();
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  const ImVec2 lo(origin.x + 4.f, origin.y + ImGui::GetTextLineHeightWithSpacing());
  const ImVec2 hi(origin.x + size.x - 4.f, origin.y + size.y - 4.f);

  const double perVolt = motor.torquePerVolt(), peak = motor.peakTorque;
  const double kv = motor.velocityConstant;
  const double xRange = 1.12 * kv * (std::max(motor.busVoltage, fileVoltage) + peak / perVolt);
  const double yRange = 1.4 * peak;
  const auto toScreen = [&](double speed, double torque) {
    const double x = std::clamp(speed / xRange, -1., 1.), y = std::clamp(torque / yRange, -1., 1.);
    return ImVec2(float(0.5 * (lo.x + hi.x) + 0.5 * x * (hi.x - lo.x)),
                  float(0.5 * (lo.y + hi.y) - 0.5 * y * (hi.y - lo.y)));
  };

  const ImU32 axisColor = IM_COL32(120, 125, 135, 120), labelColor = IM_COL32(150, 155, 165, 255);
  draw->AddRectFilled(lo, hi, IM_COL32(22, 25, 31, 235), 3.f);
  draw->AddLine(toScreen(-xRange, 0.), toScreen(xRange, 0.), axisColor);
  draw->AddLine(toScreen(0., -yRange), toScreen(0., yRange), axisColor);

  // EM-MOR: the bus-voltage band cut by the peak torque, a parallelogram.
  const double volt = motor.busVoltage;
  const double margin = peak / perVolt;
  const ImVec2 region[4] = {
      toScreen(kv * (-volt - margin), peak), toScreen(kv * (volt - margin), peak),
      toScreen(kv * (volt + margin), -peak), toScreen(kv * (-volt + margin), -peak)};
  draw->AddConvexPolyFilled(region, 4, IM_COL32(70, 190, 105, 55));
  draw->AddPolyline(region, 4, IM_COL32(95, 215, 125, 255), ImDrawFlags_Closed, 1.5f);

  // Box-MOR: peak torque and no-load speed, dashed.
  const double noLoad = motor.noLoadSpeed();
  const ImVec2 box[4] = {toScreen(-noLoad, peak), toScreen(noLoad, peak), toScreen(noLoad, -peak),
                         toScreen(-noLoad, -peak)};
  for (int side = 0; side < 4; ++side) {
    const ImVec2 a = box[side], b = box[(side + 1) % 4];
    for (float t = 0.f; t < 1.f; t += 0.04f)
      draw->AddLine(ImVec2(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t),
                    ImVec2(a.x + (b.x - a.x) * (t + 0.02f), a.y + (b.y - a.y) * (t + 0.02f)),
                    IM_COL32(210, 210, 210, 150));
  }

  const size_t shown = std::min(trail, trace.count);
  for (size_t i = 0; i < shown; ++i) {
    const size_t index = (trace.head + trace.ring.size() - shown + i) % trace.ring.size();
    const Sample& sample = trace.ring[index];
    const ImVec2 p = toScreen(sample.speed, sample.torque);
    const float r = sample.outside ? 2.5f : 1.2f;
    draw->AddRectFilled(ImVec2(p.x - r, p.y - r), ImVec2(p.x + r, p.y + r), sampleColor(sample));
  }

  char text[160];
  std::snprintf(text, sizeof(text), "%s  max out %.2f%%  sat %.0f%%", name.c_str(),
                100. * trace.maxOutside / peak,
                trace.samples ? 100. * double(trace.saturated) / double(trace.samples) : 0.);
  draw->AddText(origin, trace.outside ? IM_COL32(255, 110, 110, 255) : IM_COL32(225, 228, 235, 255),
                text);
  std::snprintf(text, sizeof(text), "%.0f rad/s", xRange);
  draw->AddText(ImVec2(hi.x - ImGui::CalcTextSize(text).x - 2.f,
                       hi.y - ImGui::GetTextLineHeight() - 1.f), labelColor, text);
  std::snprintf(text, sizeof(text), "%.1f Nm", yRange);
  draw->AddText(ImVec2(0.5f * (lo.x + hi.x) + 3.f, lo.y + 1.f), labelColor, text);
  ImGui::Dummy(size);
}

void positionCamera(raisin::RayraiWindow& viewer) {
  auto& camera = viewer.getCamera();
  // The plots cover the right of the window; aim so that the rig sits in the free lower left.
  camera.position = glm::vec3(1.64f, -3.55f, 1.57f);
  camera.target = glm::vec3(1.445f, 0.44f, 1.374f);
  const auto direction = glm::normalize(camera.target - camera.position);
  camera.yaw = glm::degrees(std::atan2(direction.y, direction.x));
  camera.pitch = glm::degrees(std::asin(direction.z));
  camera.zoom = 45.f;
  camera.zNear = 0.03f;
  camera.zFar = 60.f;
  camera.setCameraFixedTarget(true);
  camera.setCameraFixedDistance(true);
  camera.update(false);
}

}  // namespace

int main(int, char** argv) {
  auto world = std::make_shared<raisim::World>();
  world->setTimeStep(0.001);
  world->addGround(0., "ground")->setAppearance("checkerboard");
  auto* robot = world->addArticulatedSystem(
      rayraiRscPath(argv[0], "motorOperatingRegion/quadruped_rig.urdf"));
  RandomActuator actuator(robot);

  const auto& names = robot->getMotorNames();
  const auto& motors = robot->getMotorParameters();
  std::vector<double> fileVoltages;
  for (const auto& motor : motors) fileVoltages.push_back(motor.busVoltage);
  std::vector<MotorTrace> traces(motors.size());
  const auto resetTraces = [&] { for (auto& trace : traces) trace.reset(); };

  ExampleApp app;
  if (!app.init("RaiSim motor operating region", 1600, 900)) return -1;
  ImGui::GetIO().IniFilename = nullptr;  // keep ImGui scratch files out of the source tree

  auto viewer = std::make_shared<raisin::RayraiWindow>(world, 1600, 900);
  viewer->setRenderQualitySettings(raisin::RayraiWindow::defaultRenderQualitySettings(
      raisin::RayraiWindow::RenderQualityPreset::Balanced));
  raisim_examples::setRayraiBackgroundColorRgb255(*viewer, {30, 34, 42, 255});
  raisim_examples::addRayraiBasicSceneLights(*viewer);
  positionCamera(*viewer);

  bool paused = false, enforce = true;
  int mode = int(actuator.mode), timeStep = 0;
  float busVoltage = float(motors[0].busVoltage);
  double accumulator = 0.;
  auto previous = std::chrono::steady_clock::now();

  while (!app.quit) {
    app.processEvents();
    if (app.quit) break;

    // Simulate in real time.
    const auto now = std::chrono::steady_clock::now();
    const double elapsed = std::chrono::duration<double>(now - previous).count();
    if (!paused) accumulator += std::min(0.05, elapsed);
    previous = now;
    while (accumulator >= world->getTimeStep()) {
      actuator.update(world->getWorldTime());
      world->integrate();
      const auto& states = robot->getMotorStates();
      for (size_t m = 0; m < states.size(); ++m) traces[m].add(states[m], motors[m]);
      accumulator -= world->getTimeStep();
    }

    app.beginFrame();
    app.renderViewer(*viewer);

    ImGui::SetNextWindowPos(ImVec2(12, 12), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowBgAlpha(0.88f);
    ImGui::Begin("Motor operating region", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
    if (ImGui::Combo("Actuation", &mode,
                     "Mixed\0Random position targets\0Random velocity sweeps\0Random torques\0")) {
      actuator.mode = static_cast<Mode>(mode);
      actuator.restart();
    }
    ImGui::SliderFloat("Command period [s]", &actuator.period, 0.05f, 2.f, "%.2f");
    ImGui::SliderFloat("Command scale", &actuator.scale, 0.1f, 2.f, "%.2f");
    if (ImGui::Combo("Time step", &timeStep, "1 ms\0002.5 ms\0005 ms\0")) {
      world->setTimeStep(timeStep == 0 ? 0.001 : timeStep == 1 ? 0.0025 : 0.005);
      resetTraces();
    }
    if (ImGui::SliderFloat("Bus voltage [V]", &busVoltage, 6.f, 36.f, "%.1f")) {
      robot->setBusVoltage(busVoltage);  // all motors at once; the regions scale with it
      resetTraces();
    }
    if (ImGui::Checkbox("Enforce EM-MOR", &enforce)) {
      robot->setMotorOperatingRegionEnforced(enforce);
      resetTraces();
    }
    ImGui::SameLine();
    ImGui::Checkbox("Paused", &paused);
    if (ImGui::Button("Reset statistics")) resetTraces();
    ImGui::Separator();
    double worst = 0.;
    size_t outside = 0, samples = 0;
    for (size_t m = 0; m < traces.size(); ++m) {
      worst = std::max(worst, traces[m].maxOutside / motors[m].peakTorque);
      outside += traces[m].outside;
      samples += traces[m].samples;
    }
    ImGui::Text("%s: worst point %.2f%% of peak torque outside", enforce ? "EM-MOR" : "Box-MOR",
                100. * worst);
    ImGui::TextColored(outside ? ImVec4(1.f, 0.45f, 0.45f, 1.f) : ImVec4(0.55f, 0.9f, 0.6f, 1.f),
                       "%zu of %zu samples outside by > %.0f%%", outside, samples,
                       100. * kTolerance);
    ImGui::TextDisabled("Green: EM-MOR. Dashed: Box-MOR (peak torque, no-load speed).\n"
                        "Points: blue inside, orange at the peak torque,\n"
                        "purple at the voltage limit, red outside.");
    ImGui::End();

    const ImVec2 display = ImGui::GetIO().DisplaySize;
    const float plotWidth = std::min(980.f, display.x * 0.62f);
    ImGui::SetNextWindowPos(ImVec2(display.x - plotWidth - 10.f, 10.f), ImGuiCond_Always);
    ImGui::SetNextWindowSize(ImVec2(plotWidth, display.y - 20.f), ImGuiCond_Always);
    ImGui::SetNextWindowBgAlpha(0.82f);
    ImGui::Begin("Motor torque [Nm] vs. motor speed [rad/s]", nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
    const int columns = 3, rows = int((names.size() + columns - 1) / columns);
    const ImVec2 available = ImGui::GetContentRegionAvail();
    const ImVec2 cell((available.x - (columns - 1) * ImGui::GetStyle().ItemSpacing.x) / columns,
                      (available.y - (rows - 1) * ImGui::GetStyle().ItemSpacing.y) / rows);
    const size_t trail = size_t(kTrailSeconds / world->getTimeStep());
    for (size_t m = 0; m < names.size(); ++m) {
      if (m % columns != 0) ImGui::SameLine();
      drawMotorPlot(names[m], motors[m], fileVoltages[m], traces[m], trail, cell);
    }
    ImGui::End();
    app.endFrame();
  }

  viewer.reset();
  app.shutdown();
  return 0;
}
