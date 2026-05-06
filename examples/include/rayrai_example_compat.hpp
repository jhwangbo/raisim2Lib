#pragma once

#include <type_traits>
#include <utility>

#include <glm/glm.hpp>

#include "rayrai/RayraiWindow.hpp"

namespace raisim_examples {
namespace rayrai_detail {

template <typename T>
class HasBackgroundColorRgb255 {
  template <typename U>
  static auto test(int) -> decltype(
    std::declval<U&>().setBackgroundColorRgb255(std::declval<glm::vec4>()), std::true_type{});

  template <typename>
  static std::false_type test(...);

 public:
  static constexpr bool value = decltype(test<T>(0))::value;
};

template <typename...>
using VoidT = void;

template <typename T, typename = void>
struct HasAdditionalLight : std::false_type {};

template <typename T>
struct HasAdditionalLight<T, VoidT<typename T::AdditionalLight>> : std::true_type {};

template <typename Viewer>
typename std::enable_if<HasBackgroundColorRgb255<Viewer>::value>::type setBackgroundRgb255(
  Viewer& viewer, const glm::vec4& color) {
  viewer.setBackgroundColorRgb255(color);
}

template <typename Viewer>
typename std::enable_if<!HasBackgroundColorRgb255<Viewer>::value>::type setBackgroundRgb255(
  Viewer& viewer, const glm::vec4& color) {
  viewer.setBackgroundColor(color);
}

template <typename Viewer>
typename std::enable_if<HasAdditionalLight<Viewer>::value>::type addBasicSceneLights(Viewer& viewer) {
  typename Viewer::AdditionalLight fillLight;
  fillLight.type = raisin::LightType::DIRECTIONAL;
  fillLight.direction = glm::normalize(glm::vec3(0.45f, -0.25f, -0.85f));
  fillLight.diffuse = glm::vec3(0.12f, 0.14f, 0.18f);
  fillLight.specular = glm::vec3(0.04f);
  viewer.addAdditionalLight(fillLight);

  typename Viewer::AdditionalLight rimLight;
  rimLight.type = raisin::LightType::POINT;
  rimLight.position = glm::vec3(-2.0f, -3.0f, 2.5f);
  rimLight.diffuse = glm::vec3(0.18f, 0.10f, 0.04f);
  rimLight.specular = glm::vec3(0.05f);
  rimLight.linear = 0.08f;
  rimLight.quadratic = 0.025f;
  viewer.addAdditionalLight(rimLight);
}

template <typename Viewer>
typename std::enable_if<!HasAdditionalLight<Viewer>::value>::type addBasicSceneLights(Viewer&) {}

template <typename Viewer>
typename std::enable_if<HasAdditionalLight<Viewer>::value>::type addPbrSceneLights(Viewer& viewer) {
  typename Viewer::AdditionalLight rimLight;
  rimLight.type = raisin::LightType::POINT;
  rimLight.position = glm::vec3(-2.0f, -4.0f, 3.0f);
  rimLight.diffuse = glm::vec3(0.10f, 0.12f, 0.18f);
  rimLight.specular = glm::vec3(0.08f);
  rimLight.linear = 0.06f;
  rimLight.quadratic = 0.02f;
  viewer.addAdditionalLight(rimLight);
}

template <typename Viewer>
typename std::enable_if<!HasAdditionalLight<Viewer>::value>::type addPbrSceneLights(Viewer&) {}

}  // namespace rayrai_detail

inline void setRayraiBackgroundColorRgb255(raisin::RayraiWindow& viewer, const glm::vec4& color) {
  rayrai_detail::setBackgroundRgb255(viewer, color);
}

inline void addRayraiBasicSceneLights(raisin::RayraiWindow& viewer) {
  rayrai_detail::addBasicSceneLights(viewer);
}

inline void addRayraiPbrSceneLights(raisin::RayraiWindow& viewer) {
  rayrai_detail::addPbrSceneLights(viewer);
}

}  // namespace raisim_examples
