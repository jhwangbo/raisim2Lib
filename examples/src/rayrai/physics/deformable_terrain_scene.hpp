// Copyright (c) 2026 Raion Robotics Inc. All rights reserved.
#pragma once

#include "rayrai/RayraiWindow.hpp"

namespace deformable_terrain {

inline void configureLighting(raisin::RayraiWindow& viewer) {
  auto quality = raisin::RayraiWindow::defaultRenderQualitySettings(raisin::RayraiWindow::RenderQualityPreset::Balanced);
  quality.highFidelityPbr = true;
  quality.gamma = 2.2f;
  quality.colorMode = raisin::ViewerColorMode::AcesApprox;
  quality.pbrToneMapping = true;
  quality.pbrExposure = 1.1f;
  quality.pbrEnvironmentIntensity = 1.8f;
  quality.pbrEnvironmentLightingTint = glm::vec3(1.0f);
  quality.mainLightDirection = glm::normalize(glm::vec3(-0.45f, 0.60f, -0.50f));
  quality.mainLightDiffuse = glm::vec3(1.0f, 0.96f, 0.90f) * 3.0f;
  quality.mainLightSpecular = glm::vec3(3.0f);
  quality.mainLightAmbient = glm::vec3(0.55f, 0.60f, 0.68f);
  quality.reflectiveGround = false;
  quality.depthOfFieldEnabled = false;
  quality.proceduralSkyBackgroundEnabled = true;
  viewer.setRenderQualitySettings(quality);
  // Sky irradiance lights the metal wishbones and dampers under the bodywork.
  viewer.generateWeatherSkyEnvironment(128, 32, true);
}

}  // namespace deformable_terrain
