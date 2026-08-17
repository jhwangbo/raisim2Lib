#pragma once

#include <cstddef>

// Keep PyTorch C++ types and standard-library types out of this interface.
// PyTorch wheels and raisim can use different libstdc++ ABIs.
extern "C" {

void *raisimgym_torch_policy_create(const char *modulePath);
void raisimgym_torch_policy_destroy(void *policy);
bool raisimgym_torch_policy_forward(void *policy,
                                    const float *observations,
                                    std::size_t numEnvs,
                                    std::size_t observationDim,
                                    float *actions,
                                    std::size_t actionDim);
const char *raisimgym_torch_policy_error();

}
