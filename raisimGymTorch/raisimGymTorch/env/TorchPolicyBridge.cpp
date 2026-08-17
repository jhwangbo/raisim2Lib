#include "TorchPolicyBridge.hpp"

#include <torch/script.h>
#include <torch/torch.h>

#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>

namespace {

struct TorchPolicy {
  explicit TorchPolicy(const char *modulePath)
      : module(torch::jit::load(std::string(modulePath))) {
    module.eval();
    module.to(at::kCPU);
  }

  torch::jit::Module module;
};

thread_local std::string lastError;

template <typename Function>
bool reportErrors(Function &&function) noexcept {
  try {
    function();
    lastError.clear();
    return true;
  } catch (const std::exception &error) {
    lastError = error.what();
  } catch (...) {
    lastError = "unknown PyTorch C++ error";
  }
  return false;
}

}  // namespace

extern "C" void *raisimgym_torch_policy_create(const char *modulePath) {
  void *policy = nullptr;
  reportErrors([&] { policy = new TorchPolicy(modulePath); });
  return policy;
}

extern "C" void raisimgym_torch_policy_destroy(void *policy) {
  delete static_cast<TorchPolicy *>(policy);
}

extern "C" bool raisimgym_torch_policy_forward(
    void *policy, const float *observations, std::size_t numEnvs,
    std::size_t observationDim, float *actions, std::size_t actionDim) {
  return reportErrors([&] {
    if (policy == nullptr)
      throw std::runtime_error("Torch policy is not loaded");

    torch::InferenceMode inferenceMode;
    auto observationTensor = torch::from_blob(
        const_cast<float *>(observations),
        {static_cast<int64_t>(numEnvs),
         static_cast<int64_t>(observationDim)},
        torch::TensorOptions().dtype(torch::kFloat32));
    auto actionTensor = static_cast<TorchPolicy *>(policy)
                            ->module.forward({observationTensor})
                            .toTensor()
                            .contiguous();
    if (!actionTensor.device().is_cpu() ||
        actionTensor.scalar_type() != torch::kFloat32 ||
        actionTensor.dim() != 2 ||
        actionTensor.size(0) != static_cast<int64_t>(numEnvs) ||
        actionTensor.size(1) != static_cast<int64_t>(actionDim)) {
      throw std::runtime_error(
          "policy must return CPU float32 actions with shape "
          "[num_envs, action_dim]");
    }
    std::memcpy(actions, actionTensor.data_ptr<float>(),
                numEnvs * actionDim * sizeof(float));
  });
}

extern "C" const char *raisimgym_torch_policy_error() {
  return lastError.c_str();
}
