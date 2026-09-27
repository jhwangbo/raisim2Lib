#pragma once
#include <array>
#include <filesystem>
#include "raisim/Rasset.hpp"
#include <memory>
#include "rayrai/example_common.hpp"
#include "rayrai/Visuals.hpp"
#include "forest_scene.hpp"

// Scene setup shared by the interactive example and the separate GPU test runner.
struct ForestViewer {
  using Viewer = raisin::RayraiWindow;
  std::shared_ptr<raisim::World> world = std::make_shared<raisim::World>();
  std::shared_ptr<Viewer> viewer;
  std::vector<std::shared_ptr<raisin::InstancedVisuals>> foliage, rockVisuals;
  size_t collisionProxyCount = 0;
  ForestViewer(const std::filesystem::path& assets, int width, int height) {
    namespace fs = std::filesystem;
    const char* names[] = {"pine_sapling_small", "fir_sapling", "tree_small_02",
      "grass_bermuda_01", "grass_medium_01", "grass_medium_02",
      "fern_02", "dandelion_01", "nettle_plant", "periwinkle_plant"};
    std::array<raisim::Rasset, 3> treeAssets;
    std::array<raisim::Rasset, 6> rockAssets;
    for (int type = 0; type < 10; ++type) {
      if (type < 3) treeAssets[type] = raisim::Rasset::load(assets/names[type]/"model.rasset");
      else if (!fs::exists(assets/names[type]/"model.gltf"))
        throw std::runtime_error("Missing forest asset: " + (assets/names[type]).string());
    }
    for (int type = 0; type < 6; ++type)
      rockAssets[type] = raisim::Rasset::load(assets/("rock_moss_" + std::to_string(type))/"model.rasset");
    auto* terrain = forest::addTerrain(*world);
    const auto plants = forest::scatter(*terrain);
    forest::addObjects(*world, *terrain, plants);
    const auto rocks = forest::scatterRocks(*terrain, plants);
    for (const auto& p : plants) if (p.type < 3)
      collisionProxyCount += treeAssets[p.type].addStaticColliders(
        *world, {p.x, p.y, p.z}, p.yaw, p.scale).size();
    for (const auto& p : rocks)
      collisionProxyCount += rockAssets[p.type].addStaticColliders(
        *world, {p.x, p.y, p.z}, p.yaw, p.scale).size();
    viewer = std::make_shared<Viewer>(world,width,height);
    viewer->setAsyncMeshLoadingEnabled(true);
    auto quality = Viewer::defaultRenderQualitySettings(Viewer::RenderQualityPreset::High);
    quality.gamma = 2.2f;
    quality.colorMode = raisin::ViewerColorMode::AcesApprox;
    quality.pbrExposure = 1.1f;
    quality.pbrEnvironmentLightingTint = glm::vec3(.6f);
    quality.depthOfFieldEnabled = false;
    quality.reflectiveGround = false;
    quality.viewerMsaaSamples = 4;
    quality.directionalShadowCascadeCount = 3;
    quality.directionalShadowCascadeMaxDistance = 80;
    quality.addViewerFillLights = false;
    viewer->setRenderQualitySettings(quality);
    Viewer::WeatherSettings weather;
    weather.enabled = true;
    weather.preset = Viewer::WeatherPreset::Clear;
    weather.useExplicitSunAngles = true;
    weather.sunAzimuthDegrees = 210;
    weather.sunElevationDegrees = 48;
    weather.sunDiskSize = .001f; // Compact sun disk in the baked sky background.
    weather.windSpeed = 1.2f;
    weather.cloudCoverage = .06f;
    // Gentle distance haze keeps nearby leaves crisp and separates the far canopy.
    weather.visibilityMeters = 1200.f;
    weather.fogColor = glm::vec3(.65f,.70f,.75f);
    viewer->setWeatherSettings(weather);
    auto sky = viewer->generateWeatherSkyEnvironment(256,32,false);
    viewer->setEnvironmentBackground(sky.environmentMap,1);
    auto& sun = viewer->getLight();
    sun.type = raisin::LightType::DIRECTIONAL;
    sun.setShadowsEnabled(true);
    sun.setShadowResolution(2048);
    sun.setShadowParams(.0008f,1.f,1.25f);
    viewer->setShadowOrtho(60,.1f,180);
    sun.direction = glm::normalize(glm::vec3(-.4f,.3f,-.85f));
    sun.ambient = glm::vec3(.12f);
    // Strong direct sun pushes exposed leaves into the ACES highlight shoulder.
    sun.diffuse = glm::vec3(1,.95f,.85f)*18.f;
    sun.specular = glm::vec3(18);
    viewer->setHeightmapPatternResourcePath((assets/"ground/mud_forest_diff_1k.jpg").string());
    viewer->setHeightmapNormalResourcePath((assets/"ground/mud_forest_nor_gl_1k.jpg").string());
    viewer->setHeightmapTerrainMaterialParameters(.45f,.98f,0.f,false);
    for (int type = 0; type < 10; ++type) {
      auto visual = viewer->addInstancedVisuals(names[type],raisim::Shape::Mesh,glm::vec3(1),
        glm::vec4(1),glm::vec4(.85f,.92f,.75f,1),(type < 3 ? treeAssets[type].visualMesh : assets/names[type]/"model.gltf").string(),true);
      std::vector<raisin::InstancedVisuals::InstanceSpec> instances;
      for (const auto& p : plants) if (p.type == type) {
        raisin::InstancedVisuals::InstanceSpec instance;
        instance.position = {p.x,p.y,p.z};
        // Rayrai's vec4 instance orientation is WXYZ.
        instance.orientation = {std::cos(p.yaw*.5),0,0,std::sin(p.yaw*.5)};
        instance.scale = glm::vec3(p.scale);
        instance.colorWeight = float((p.scale/std::max(1.,p.scale))*.4);
        instances.push_back(instance);
      }
      visual->addInstances(instances);
      foliage.push_back(visual);
      visual->setAutomaticMeshLodEnabled(true);
      visual->setProjectedLodPolicy(true,2,8);
      visual->setShadowFoliageLodPolicy(true,2.5f,16);
      visual->setSortTransparentInstances(false);
      visual->configureFoliageWind(0,type < 3 ? 1.3f : .5f,type < 3 ? .025f : .09f,.7f,.2f);
      visual->setFoliageImpostorPolicy(false);
    }
    for (int type=0;type<6;++type) {
      const auto name = "rock_moss_" + std::to_string(type);
      const auto& file = rockAssets[type].visualMesh;
      auto visual = viewer->addInstancedVisuals(name,raisim::Shape::Mesh,glm::vec3(1),
        glm::vec4(1),glm::vec4(1),file.string(),true);
      std::vector<raisin::InstancedVisuals::InstanceSpec> instances;
      for (const auto& p:rocks) if (p.type==type) {
        raisin::InstancedVisuals::InstanceSpec instance;
        instance.position={p.x,p.y,p.z};
        instance.orientation={std::cos(p.yaw*.5),0,0,std::sin(p.yaw*.5)};
        instance.scale=glm::vec3(p.scale);
        instances.push_back(instance);
      }
      visual->addInstances(instances);
      visual->setAutomaticMeshLodEnabled(true);
      rockVisuals.push_back(visual);
    }
    auto& camera = viewer->getCamera();
    const double eyeX = forest::trail(-23);
    camera.position = {float(eyeX),-23,float(terrain->getHeight(eyeX,-23)+2.6)};
    camera.yaw = 79;
    camera.pitch = -8;
    camera.zoom = 55;
  }
};
