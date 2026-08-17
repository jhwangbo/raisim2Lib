//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <chrono>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <type_traits>
#ifdef RAISIMGYM_TORCH_WITH_LIBTORCH
#include <torch/script.h>
#include <torch/torch.h>
#endif
#include "Environment.hpp"
#include "VectorizedEnvironment.hpp"

namespace py = nanobind;
namespace nb = nanobind;
using namespace raisim;
int THREAD_COUNT = 1;

template<typename Environment, typename = void>
struct ParallelizeVisualEnvironment : std::false_type {};

template<typename Environment>
struct ParallelizeVisualEnvironment<
    Environment,
    std::void_t<decltype(Environment::kParallelizeVisualEnvironment)>>
    : std::bool_constant<Environment::kParallelizeVisualEnvironment> {};

#ifndef ENVIRONMENT_NAME
  #define ENVIRONMENT_NAME RaisimGymEnv
#endif

NB_MODULE(RAISIMGYM_TORCH_ENV_NAME, m) {
  using EnvType = VectorizedEnvironment<ENVIRONMENT,
                                        ENVIRONMENT::kStaticSchedule,
                                        ParallelizeVisualEnvironment<ENVIRONMENT>::value>;
  py::class_<EnvType>(m, RSG_MAKE_STR(ENVIRONMENT_NAME))
    .def(py::init<std::string, std::string>(), py::arg("resourceDir"), py::arg("cfg"))
    .def("init", &EnvType::init)
    .def("reset", &EnvType::reset)
    .def("observe", &EnvType::observe)
    .def("observeCritic", &EnvType::observeCritic)
    .def("step", &EnvType::step)
    .def("stepAndObserve", &EnvType::stepAndObserve)
    .def("setSeed", &EnvType::setSeed)
    .def("getRewardInfo", &EnvType::getRewardInfo, py::rv_policy::reference_internal)
    .def("getRewardNames", &EnvType::getRewardNames)
    .def("close", &EnvType::close)
    .def("isTerminalState", &EnvType::isTerminalState)
    .def("setSimulationTimeStep", &EnvType::setSimulationTimeStep)
    .def("setControlTimeStep", &EnvType::setControlTimeStep)
    .def("getObDim", &EnvType::getObDim)
    .def("getCriticObDim", &EnvType::getCriticObDim)
    .def("getActionDim", &EnvType::getActionDim)
    .def("getNumOfEnvs", &EnvType::getNumOfEnvs)
    .def("getNumThreads", &EnvType::getNumThreads)
    .def("setNumThreads", &EnvType::setNumThreads)
    .def("turnOnVisualization", &EnvType::turnOnVisualization)
    .def("turnOffVisualization", &EnvType::turnOffVisualization)
    .def("stopRecordingVideo", &EnvType::stopRecordingVideo)
    .def("startRecordingVideo", &EnvType::startRecordingVideo)
    .def("curriculumUpdate", &EnvType::curriculumUpdate)
    .def("getObStatistics", &EnvType::getObStatistics)
    .def("setObStatistics", &EnvType::setObStatistics)
    .def("__getstate__", [](const EnvType &p) {
        return py::make_tuple(p.getResourceDir(), p.getCfgString());
    })
    .def("__setstate__", [](EnvType &p, py::tuple t) {
        if (t.size() != 2) {
            throw std::runtime_error("Invalid state!");
        }

        new (&p) EnvType(py::cast<std::string>(t[0]),
                                                    py::cast<std::string>(t[1]));
    });

  py::class_<NormalSampler>(m, "NormalSampler")
    .def(py::init<int>(), py::arg("dim"))
    .def("seed", &NormalSampler::seed)
    .def("sample", &NormalSampler::sample);

  // Preserve NumPy's float64 GAE recurrence while moving its 400 small Python
  // iterations into one native call. Normalization remains in RolloutStorage.
  m.def("compute_returns",
        [](nb::ndarray<const float, nb::ndim<3>, nb::c_contig, nb::device::cpu> rewards,
           nb::ndarray<const bool, nb::ndim<3>, nb::c_contig, nb::device::cpu> dones,
           nb::ndarray<const float, nb::ndim<3>, nb::c_contig, nb::device::cpu> values,
           nb::ndarray<const float, nb::ndim<2>, nb::c_contig, nb::device::cpu> lastValues,
           nb::ndarray<float, nb::ndim<3>, nb::c_contig, nb::device::cpu> returns,
           double gamma,
           double lambda) {
          const size_t steps = rewards.shape(0);
          const size_t envs = rewards.shape(1);
          if (rewards.shape(2) != 1 || dones.shape(0) != steps ||
              dones.shape(1) != envs || dones.shape(2) != 1 ||
              values.shape(0) != steps || values.shape(1) != envs ||
              values.shape(2) != 1 || returns.shape(0) != steps ||
              returns.shape(1) != envs || returns.shape(2) != 1 ||
              lastValues.shape(0) != envs || lastValues.shape(1) != 1) {
            throw std::runtime_error("invalid generalized-advantage array shape");
          }

          const float *rewardData = rewards.data();
          const bool *doneData = dones.data();
          const float *valueData = values.data();
          const float *lastValueData = lastValues.data();
          float *returnData = returns.data();
          nb::gil_scoped_release release;
#pragma omp parallel for schedule(static)
          for (size_t env = 0; env < envs; ++env) {
            double advantage = 0.0;
            for (size_t step = steps; step-- > 0;) {
              const size_t index = step * envs + env;
              const double nextValue = step + 1 == steps
                                           ? static_cast<double>(lastValueData[env])
                                           : static_cast<double>(valueData[index + envs]);
              const double notTerminal = doneData[index] ? 0.0 : 1.0;
              const double discounted = (notTerminal * gamma) * nextValue;
              const double delta =
                  (static_cast<double>(rewardData[index]) + discounted) -
                  static_cast<double>(valueData[index]);
              const double trace = ((notTerminal * gamma) * lambda) * advantage;
              advantage = delta + trace;
              returnData[index] = static_cast<float>(
                  advantage + static_cast<double>(valueData[index]));
            }
          }
        },
        py::arg("rewards"), py::arg("dones"), py::arg("values"),
        py::arg("last_values"), py::arg("returns"), py::arg("gamma"),
        py::arg("lambda"));

#ifdef RAISIMGYM_TORCH_WITH_LIBTORCH
  using nb::c_contig;
  using nb::device::cpu;

  class TorchPolicyRunner {
   public:
    using RewardArray =
        nb::ndarray<float, nb::ndim<3>, c_contig, cpu>;

    TorchPolicyRunner(EnvType &env, const std::string &modulePath)
        : env_(env),
          observations_(env.getNumOfEnvs(), env.getObDim()),
          rewards_(env.getNumOfEnvs()),
          dones_(env.getNumOfEnvs()) {
      load(modulePath);
    }

    void load(const std::string &modulePath) {
      module_ = torch::jit::load(modulePath);
      module_.eval();
      module_.to(at::kCPU);
    }

    void run(RewardArray rewardInformation, double controlDt = 0.0) {
      const size_t stepCount = rewardInformation.shape(0);
      const size_t numEnvs = static_cast<size_t>(env_.getNumOfEnvs());
      const size_t observationDim = static_cast<size_t>(env_.getObDim());
      const size_t actionDim = static_cast<size_t>(env_.getActionDim());
      const size_t rewardDim = static_cast<size_t>(env_.getRewardInfo().cols());
      if (rewardInformation.shape(1) != numEnvs ||
          rewardInformation.shape(2) != rewardDim) {
        throw std::runtime_error(
            "reward_information must have shape [steps, num_envs, reward_dim]");
      }

      float *rewardInfo = rewardInformation.data();
      const size_t valuesPerStep = numEnvs * rewardDim;
      const auto frameDuration = std::chrono::duration<double>(controlDt);
      nb::gil_scoped_release release;
      torch::NoGradGuard noGrad;
      for (size_t step = 0; step < stepCount; ++step) {
        const auto frameStart = std::chrono::steady_clock::now();
        env_.observe(observations_, false);
        auto observationTensor = torch::from_blob(
            observations_.data(),
            {static_cast<int64_t>(numEnvs),
             static_cast<int64_t>(observationDim)},
            torch::TensorOptions().dtype(torch::kFloat32));
        auto actions = module_.forward({observationTensor}).toTensor().contiguous();
        if (!actions.device().is_cpu() || actions.scalar_type() != torch::kFloat32 ||
            actions.dim() != 2 || actions.size(0) != static_cast<int64_t>(numEnvs) ||
            actions.size(1) != static_cast<int64_t>(actionDim)) {
          throw std::runtime_error(
              "policy must return CPU float32 actions with shape [num_envs, action_dim]");
        }
        Eigen::Map<EigenRowMajorMat> actionMap(
            actions.data_ptr<float>(), numEnvs, actionDim);
        env_.step(actionMap, rewards_, dones_);
        std::memcpy(rewardInfo + step * valuesPerStep,
                    env_.getRewardInfo().data(),
                    valuesPerStep * sizeof(float));
        if (controlDt > 0.0) {
          const auto elapsed = std::chrono::steady_clock::now() - frameStart;
          if (elapsed < frameDuration)
            std::this_thread::sleep_for(frameDuration - elapsed);
        }
      }
    }

   private:
    EnvType &env_;
    torch::jit::Module module_;
    EigenRowMajorMat observations_;
    EigenVec rewards_;
    EigenBoolVec dones_;
  };

  py::class_<TorchPolicyRunner>(m, "TorchPolicyRunner")
    .def(py::init<EnvType &, const std::string &>(),
         py::arg("env"), py::arg("module_path"), py::keep_alive<1, 2>())
    .def("load", &TorchPolicyRunner::load, py::arg("module_path"))
    .def("run", &TorchPolicyRunner::run,
         py::arg("reward_information"), py::arg("control_dt") = 0.0);
#endif
}
